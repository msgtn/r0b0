const bcrypt = require('bcryptjs');
const hashUtils = require('./hash-utils');
const XError = require('xerror');

/**
 * Utility functions for checking passwords against hashes.
 * @name checkUtils
 * @kind module
 * @private
 */

 /**
  * Examines the prefix of a hash string to determine its type.
  * @memberof checkUtils
  * @param {String} hash - Hash string from htpasswd
  * @returns {String} - 'bcrypt', 'md5', 'sha1' or 'crypt'.
  */
exports.getHashType = function(hash) {
	if (/^\$2.?\$/.test(hash)) return 'bcrypt';
	if (/^\$(apr)?1\$/.test(hash)) return 'md5';
	if (/^\{SHA\}/.test(hash)) return 'sha1';
	return 'crypt';
};

/**
 * Gets a password hashing function for the provided hash type. `bcrypt` is not
 * supported, because the bcryptjs library already provides password-checking
 * functions, so performing bcrypt hashes ourselves is not necessary.
 * @memberof checkUtils
 * @param {String} hashType - 'md5', 'sha1', or 'crypt'
 * @returns {Function} - Accepts two arguments-- password and hash. The hash
 *   argument contains the salt, if any is to be used.
 */
exports.getHashFunction = function(hashType) {
	let fn = hashUtils[hashType];
	if (fn) return fn;
	throw new XError(
		XError.INVALID_ARGUMENT,
		`Unsupported hash type '${hashType}'`,
		{ hashType }
	);
};

/**
 * Gets a password-checking function for the provided hash type, wrapping the
 * result in a promise. This is to provide a consistent asynchronous interface,
 * even though actual asynchronous code will only occur in the case of 'bcrypt'.
 * 'bcrypt' itself is not supported here, because the bcryptjs library already
 * provides a password-checking function. Creating it ourselves is not
 * necessary.
 * @memberof checkUtils
 * @param {String} hashType - 'md5', 'sha1', or 'crypt'
 * @returns {Function} - Accepts two arguments-- password and hash. Returns
 *   a promise that will resolve with true if the password is correct, or false
 *   otherwise.
 */
exports.createCheckFunction = function(hashType) {
	let hashFunction = exports.getHashFunction(hashType);
	return (pass, hash) => Promise.resolve(hashFunction(pass, hash) === hash);
};

/**
 * Gets a password-checking function for the provided hash, with bcrypt support.
 * @memberof checkUtils
 * @param {String} hash - Hash string from htpasswd.
 * @returns {Function} - Accepts two arguments-- password and hash. Returns
 *   a promise that will resolve with true if the password is correct, or false
 *   otherwise.
 */
exports.getCheckFunction = function(hash) {
	let hashType = exports.getHashType(hash);
	if (hashType === 'bcrypt') return bcrypt.compare;
	return exports.createCheckFunction(hashType);
};

/**
 * Checks a password against the provided hash.
 * @memberof checkUtils
 * @param {String} password - Password to check.
 * @param {String} hash - Hash string from htpasswd.
 * @returns {Promise<boolean>} - Will resolve with true if the password
 *   was correct, false otherwise.
 */
exports.checkPassword = function(password, hash) {
	return exports.getCheckFunction(hash)(password, hash);
};
