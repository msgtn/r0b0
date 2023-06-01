var registry = require('./error-code-registry');
require('./common-error-codes')();

/**
 * Construct a Extended Error instance.
 * All parameters are optional, with the exception that a code is required if a message is given, and
 * that data is required if privateData is given.
 *
 * @class XError
 * @constructor
 * @extends Error
 * @uses ErrorCodeRegistry
 * @param {String} [code="internal_error"] - The error code of the error.  These are short string codes.  They may not contain spaces.
 * @param {String} [message] - Human-readable error message.
 * @param {Object} [data] - Additional data about the error.
 * @param {Object} [privateData] - Data that should be considered private to the application and is not displayed to users.
 * @param {Error|XError} [cause] - The error which caused this error.  Used to create error cause chains.
 */
function XError(/*code, message, data, privateData, cause*/) {
	if (Error.captureStackTrace) Error.captureStackTrace(this, this);
	else this.stack = new Error().stack;

	var code, message, data, cause, privateData;

	for(var i = 0; i < arguments.length; i++) {
		var arg = arguments[i];
		if(XError.isXError(arg) || arg instanceof Error) {
			if(cause !== undefined) {
				data = cause;
			}
			cause = arg;
		} else if(typeof arg === 'string' || typeof arg === 'number') {
			if(typeof arg !== 'string') arg = '' + arg;
			if(code === undefined && arg.indexOf(' ') == -1) code = arg;
			else if(message === undefined) message = arg;
			else if(data === undefined) data = arg;
			else if(privateData === undefined) privateData = arg;
			else if(cause === undefined) cause = new Error(arg);
		} else if(arg === null || arg === undefined) {
			if(code === undefined) code = null;
			else if(message === undefined) message = null;
			else if(data === undefined) data = null;
			else if(privateData === undefined) privateData = null;
			else if(cause === undefined) cause = null;
		} else {
			if(data === undefined) data = arg;
			else if(privateData === undefined) privateData = arg;
		}
	}

	code = code || (cause && cause.code) || XError.INTERNAL_ERROR;
	message = message || (cause && cause.message);

	if(code) {
		var errorCodeInfo = registry.getErrorCode(code);
		if(errorCodeInfo) {
			if(errorCodeInfo.code) code = errorCodeInfo.code;	// in case of aliased error codes
			if(!message && errorCodeInfo.message) message = errorCodeInfo.message;
		}
	}

	this.code = code;
	this.message = message || 'An error occurred';
	this.data = data;
	this.privateData = privateData;
	this.cause = cause;
	this.name = 'XError';
	Object.defineProperty(this, '_isXError', { configurable: false, enumerable: false, writable: false, value: true });
}

// Inherit from Error
XError.super_ = Error;
XError.prototype = Object.create(Error.prototype, {
  constructor: {
    value: XError,
    enumerable: false,
    writable: true,
    configurable: true
  }
});

// Register code accessors on the error object
registry.listErrorCodes().forEach(function(code) {
	XError[code.toUpperCase()] = code;
});
registry.addXError(XError);

/**
 * Returns a string representation of the error.  This is not necessarily consumer-friendly.
 *
 * @method toString
 * @return {String} - String representation of the error
 */
XError.prototype.toString = function() {
	var str = '' + this.code + ': ' + this.message;
	if(this.data) str += ' -- ' + JSON.stringify(this.data);
	if(this.cause) str += ' -- ' + JSON.stringify(this.cause);
	return str;
};

/**
 * Converts the information inside this XError object into a plain object suitable
 * for stringifying or otherwise transmitting.
 *
 * @method toObject
 * @param {Object} [options] - Options regarding the conversion
 * @param {Boolean} [options.includePrivateData]
 * @param {Boolean} [options.includeStack]
 * @param {String[]} [options.extraFields] - Extra fields to include in the object
 * @return {Object}
 */
XError.prototype.toObject = function(options) {
	if(!options) options = {};
	var obj = {};
	var i;
	if(this.code) obj.code = this.code;
	if(this.message) obj.message = this.message;
	if(this.data) obj.data = this.data;
	if(this.privateData && options.includePrivateData) obj.privateData = this.privateData;
	if(this.stack && options.includeStack) obj.stack = this.stack;
	if(options.extraFields) {
		for(i = 0; i < options.extraFields.length; i++) {
			if(this[options.extraFields[i]]) {
				obj[options.extraFields[i]] = this[options.extraFields[i]];
			}
		}
	}
	if(this.cause) {
		if(typeof this.cause.toObject === 'function') {
			obj.cause = this.cause.toObject(options);
		} else if(this.cause.code || this.cause.message) {
			obj.cause = XError.prototype.toObject.call(this.cause, options);
		} else {
			obj.cause = {
				message: '' + this.cause
			};
		}
	}
	return obj;
};

/**
 * Instantiates and returns a new XError object from the given object.
 *
 * @method fromObject
 * @static
 * @param {Object} obj
 * @return {XError}
 */
XError.fromObject = function(obj) {
	var cause = null;
	if(obj.cause) {
		cause = XError.fromObject(obj.cause);
	}
	var xerror = new XError(obj.code || null, obj.message || null, obj.data || null, obj.privateData || null, cause || null);
	if(obj.stack) {
		xerror.stack = obj.stack;
	} else {
		delete xerror.stack;
	}
	return xerror;
};

/**
 * Returns true iff the given value is an instance of a XError.
 *
 * @method isXError
 * @static
 * @param {Mixed} o - Thing to determine if it's an XError
 * @return {Boolean}
 */
XError.isXError = function(o) {
	return typeof o == 'object' && o && o._isXError === true;
};

/**
 * Wraps an error in another XError.  If an XError is passed in, this is equivalent to:
 * new XError(error.code, message || error.message, error.data, error.privateData, error)
 * If a non-xerror is passed in, this is equivalent to:
 * new XError(XError.INTERNAL_ERROR, message || error.message || 'An error occurred', error)
 *
 * This is useful to wrap errors that occur in a deep layer with shallower-layer errors to provide a more
 * informative error message to the user.  It also has the effect of adding a stack track, which can help
 * in debugging.
 *
 * @method wrap
 * @static
 * @param {Error|ZSError} error - Error to wrap
 * @param {String} [message] - Optional message override
 * @return {ZSError} - Wrapped ZSError
 */
XError.wrap = function(error, message) {
	if(!error) return error;
	if(XError.isXError(error)) {
		return new XError(error.code, message || error.message, error.data, error.privateData, error);
	} else {
		return new XError(XError.INTERNAL_ERROR, message || error.message || 'An error occurred', error);
	}
};

XError.registerErrorCode = registry.registerErrorCode;
XError.registerErrorCodes = registry.registerErrorCodes;
XError.getErrorCode = registry.getErrorCode;
XError.listErrorCodes = registry.listErrorCodes;

XError.create = XError.wrap;

module.exports = XError;
