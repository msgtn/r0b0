var registry = require('./error-code-registry');

var codes = {
	access_denied: { message: 'Access Denied', http: 403 },
	already_exists: { message: 'Already Exists', http: 409 },
	bad_request: { message: 'Bad Request', http: 400 },
	internal_error: { message: 'Internal Error', http: 500 },
	not_found: { message: 'Not Found', http: 404 },
	unsupported_format: { message: 'Unsupported format', http: 415 },
	timed_out: { message: 'Operation timed out', http: 504 },
	unsupported_operation: { message: 'Operation not supported', http: 404 },
	limit_exceeded: { message: 'Limit exceeded', http: 503 },
	not_modified: { message: 'Not modified', http: 304 },
	invalid_argument: { message: 'Invalid data passed to function', http: 401 },
	conflict: { message: 'Data conflict', http: 409 }
};

module.exports = function() {
	registry.registerErrorCodes(codes);
};
