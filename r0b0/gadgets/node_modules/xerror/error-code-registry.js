// We store the registry on the global Error object in case there are multiple versions
// of xerror in use in a project.
var registry = Error._xerrorCodeRegistry;
if(!registry) {
	registry = { codes: {}, xerrors: [] };
	Object.defineProperty(Error, '_xerrorCodeRegistry', {
		configurable: false,
		enumerable: false,
		value: registry,
		writable: true
	});
}


/**
 * Central registry for error codes.
 *
 * @class ErrorCodeRegistry
 */


/**
 * Registers an error code with the central registry.
 *
 * @method registerErrorCode
 * @static
 * @param {String} code - The error code to register
 * @param {Object} fields - Informational fields about the error
 * @param {String} [fields.message] - Error message associated with the code
 * @param {String} [fields.aliasOf] - If this error code is an alias, this is the code it is aliased to
 * @param {Number} [fields.http] - HTTP response code associated with the error
 * @param {Number} [version] - The version of error fields; if the same code is registered multiple times,
 * the higher version takes precedence.  This defaults to 1.
 */
exports.registerErrorCode = function(code, fields, version) {
	if(!fields) {
		fields = {};
	}
	if(!version) {
		version = 1;
	}
	if(!registry.codes[code]) {
		registry.codes[code] = {};
	}
	for(var key in fields) {
		if(!registry.codes[code][key]) {
			registry.codes[code][key] = fields[key];
		} else if(registry.codes[code].version === undefined || registry.codes[code].version <= version) {
			registry.codes[code][key] = fields[key];
		}
	}
	if(registry.codes[code].version === undefined || registry.codes[code].version <= version) {
		registry.codes[code].version = version;
	}
	registry.codes[code].code = code;
	registry.xerrors.forEach(function(XError) {
		XError[code.toUpperCase()] = code;
	});
};

/**
 * Register multiple error codes at the same time.
 *
 * @method registerErrorCodes
 * @static
 * @param {Object} codes - Mapping from code strings to fields
 * @param {Number} version
 */
exports.registerErrorCodes = function(codes, version) {
	for(var code in codes) {
		exports.registerErrorCode(code, codes[code], version);
	}
};

/**
 * Fetches information about error codes.  The returned object may include the fields
 * 'code' (useful if the error code is aliased to a different code), 'message' (default
 * error message), 'http' (associated http code).  Any of these fields may be missing.
 *
 * @method getErrorCode
 * @static
 * @param {String} code - The error code to fetch
 * @return {Object} - Information about the returned error
 */
exports.getErrorCode = function(code) {
	if(registry.codes[code]) {
		if(registry.codes[code].aliasOf) {
			return exports.getErrorCode(registry.codes[code].aliasOf);
		}
		return registry.codes[code];
	} else {
		return undefined;
	}
};

/**
 * Returns a list of error code strings.
 *
 * @method listErrorCodes
 * @static
 * @return {String[]} - List of error codes
 */
exports.listErrorCodes = function() {
	return Object.keys(registry.codes);
};

/**
 * Add an XError constructor to the list.  Used to manage different versions
 * of XError.
 *
 * @method addXError
 * @static
 * @param {Function} XError - XError constructor
 */
exports.addXError = function(XError) {
	registry.xerrors.push(XError);
};
