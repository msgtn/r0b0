var expect = require('chai').expect;

var XError = require('../xerror');

describe('XError', function() {

	it('should allow construction with just a code', function(done) {
		var error = new XError('test_code');
		expect(error.code).to.equal('test_code');
		done();
	});

	it('should allow construction with just a code and a cause', function(done) {
		var cause = new Error();
		var error = new XError(XError.INTERNAL_ERROR, cause);
		expect(error.code).to.equal('internal_error');
		expect(error.message).to.equal('Internal Error');
		expect(error.cause).to.equal(cause);
		done();
	});

	it('should allow construction with just a message', function(done) {
		var error = new XError('Something bad happened');
		expect(error.code).to.equal('internal_error');
		expect(error.message).to.equal('Something bad happened');
		done();
	});

	it('should allow construction with a full list of arguments', function(done) {
		var data = { foo: 'bar' };
		var privateData = { foo: 'baz' };
		var cause = new Error();
		var error = new XError(XError.NOT_FOUND, 'Somethingmissing', data, privateData, cause);
		expect(error.code).to.equal('not_found');
		expect(error.message).to.equal('Somethingmissing');
		expect(error.data).to.equal(data);
		expect(error.privateData).to.equal(privateData);
		expect(error.cause).to.equal(cause);
		done();
	});

	it('should allow construction with no arguments', function(done) {
		var error = new XError();
		expect(error.code).to.equal('internal_error');
		expect(error.message).to.equal('Internal Error');
		expect(error.stack).to.exist;
		done();
	});

	it('should respond to isXError', function(done) {
		var error1 = new Error();
		var error2 = new XError();
		expect(XError.isXError(error1)).to.equal(false);
		expect(XError.isXError(error2)).to.equal(true);
		done();
	});

	it('should allow construction with only a cause', function(done) {
		var cause = new Error('testmessage');
		var error = new XError(cause);
		expect(error.code).to.equal('internal_error');
		expect(error.message).to.equal('testmessage');
		expect(error.cause).to.equal(cause);
		done();
	});

	it('should allow construction with an existing XError', function(done) {
		var error1 = new XError(XError.NOT_FOUND, 'notfoundmessage', { foo: 'bar' });
		var error2 = new XError(error1);
		expect(error2.code).to.equal('not_found');
		expect(error2.message).to.equal('notfoundmessage');
		expect(error2.data).to.not.exist;
		done();
	});

	it('should allow wrapping other errors', function(done) {
		var error1 = new XError(XError.NOT_FOUND, 'notfoundmessage', { foo: 'bar' });
		var error2 = XError.wrap(error1);
		expect(error2.code).to.equal('not_found');
		expect(error2.message).to.equal('notfoundmessage');
		expect(error2.data).to.deep.equal({ foo: 'bar' });
		done();
	});

	it('should be an instance of Error', function(done) {
		var error = new XError();
		expect(error instanceof Error).to.equal(true);
		done();
	});

	it('should allow registering custom error codes', function(done) {
		XError.registerErrorCode('testing_error_code', { message: 'Test Error' });
		var error = new XError(XError.TESTING_ERROR_CODE);
		expect(error.code).to.equal('testing_error_code');
		expect(error.message).to.equal('Test Error');
		done();
	});

	it('should follow aliases', function(done) {
		XError.registerErrorCode('not_found2', { aliasOf: XError.NOT_FOUND });
		var error = new XError(XError.NOT_FOUND2);
		expect(error.code).to.equal('not_found');
		expect(error.message).to.equal('Not Found');
		done();
	});

	it('should convert to and from objects', function(done) {
		var cause = new XError(XError.NOT_FOUND, 'Thingy not found', null, { privateStuff: 'foo' });
		var error = new XError(XError.INTERNAL_ERROR, cause);
		var errorObj = error.toObject({ includeStack: true });
		expect(errorObj.code).to.equal('internal_error');
		expect(errorObj.message).to.equal('Thingy not found');
		expect(errorObj.cause).to.exist;
		expect(errorObj.cause.code).to.equal('not_found');
		expect(errorObj.cause.data).to.not.exist;
		expect(errorObj.cause.privateData).to.not.exist;
		expect(errorObj.stack).to.exist;
		expect(errorObj.cause.stack).to.exist;
		var newError = XError.fromObject(errorObj);
		expect(newError.code).to.equal('internal_error');
		expect(newError.message).to.equal('Thingy not found');
		expect(newError.cause).to.exist;
		expect(newError.cause.code).to.equal('not_found');
		expect(newError.cause.data).to.not.exist;
		expect(newError.cause.privateData).to.not.exist;
		expect(newError.stack).to.equal(errorObj.stack);
		expect(newError.cause.stack).to.equal(errorObj.cause.stack);
		done();
	});
});

