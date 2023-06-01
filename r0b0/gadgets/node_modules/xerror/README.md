# XError

A utility class extending error that provides other useful features for standardized
error codes and handling.  It inherits from Error and is compatible with it.

## Error Codes

XError objects contain a machine-readable error code as well as a human-readable message.  By convention,
error codes are lowercase, underscore_separated strings, such as `internal_error`.  Registered error
codes are available as constants on the XError object, with the names uppercased.  The common error
codes are listed [here](https://github.com/crispy1989/node-xerror/blob/master/common-error-codes.js)
and you can register your own (see below).

## Data

XError objects contain both a `data` field and a `privateData` field.  These are intended to be set to
objects that contain extra data about the error.  `privateData` is used for information that should
not be displayed to a user.

## Cause

XError objects can contain a "cause" error, similar to exception chains in languages such as Java.  Causes
can be either an XError or an Error.

## Fields

* `xerror.code` - The string error code, like `internal_error`
* `xerror.message` - Human-readable error message
* `xerror.data` - Object containing extra data about the error
* `xerror.privateData` - Object containing sensitive data about the error
* `xerror.cause` - XError or Error instance that triggered this error
* `xerror.stack` - Stack trace of where this error is constructed

## Constructing

````javascript
var XError = require('xerror');

// Constructs an XError object with the NOT_FOUND error code and
// default message for that error code.
new XError(XError.NOT_FOUND);

// Constructs an XError object with just a message and the default
// code of 'internal_error'.  This is distinguished from a code
// because it contains spaces.
new XError('Something bad happened');

// Constructs an XError object with both a code and a message
// The message need not contain spaces if a code is supplied
new XError(XError.BAD_REQUEST, 'You did something wrong');

// Constructs an XError object with a cause
new XError(XError.BAD_REQUEST, 'Something was wrong with your call', causeError);

// Constructs an XError object with data
new XError(XError.INVALID_ARGUMENT, { name: 'id', value: 'somethinginvalid' });

// Constructs an XError object with everything
new XError(
	XError.ACCESS_DENIED,
	'Error validating account credentials',
	{ username: 'crispy1989' },
	{ password: 'password123' },
	new Error('validation error')
);

// Wrap an existing XError or Error
// This replicates all of the fields, but creates a new stack trace and sets
// the original error as the cause.
// This is useful when tracing errors through callback chains
XError.wrap(otherXError);
// Use it like this:
doSomething(function(error) {
	if(error) return cb(XError.wrap(error));
	// ...
});
````

## Registering custom error codes

Error codes should be registered before being used.  You can also register additional properties with the error code.

````javascript
// Register a new custom error code
XError.registerErrorCode('my_custom_code', {
	// default error message to use with this error code
	message: 'Custom Default Error Message',
	// the http response code to map from this error code
	http: 499
	// you can add other custom fields as well
});
var error = new XError(XError.MY_CUSTOM_CODE, ...);

// Register an alias of another error code
XError.registerErrorCode('item_missing', { aliasOf: XError.NOT_FOUND });

// Register multiple at once
XError.registerErrorCodes({
	custom_code_1: { message: 'Message for Custom Code 1' },
	custom_code_2: { message: 'Message for Custom Code 2' }
});
````

You can also fetch these informational object about error codes:

````javascript
var info = XError.getErrorCode(XError.NOT_FOUND);

var listOfCodes = XError.listErrorCodes();
````

