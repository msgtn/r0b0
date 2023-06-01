const crypto = require('crypto');

/**
 * Contains hashing functions for various algorithms supported by htpasswd.
 * @name hashUtils
 * @kind module
 * @private
 */

/**
 * Re-export of apache-md5 module for unit testing purposes.
 * See https://www.npmjs.com/package/apache-md5
 * @kind function
 * @param {String} password - Password to hash.
 * @param {String} [encryptedPassword] - Contains salt. Salt will be randomly
 *   generated, if not provided.
 * @returns {String} - Hashed password.
 */
exports.md5 = require('apache-md5');

/**
 * Re-export of apache-crypt module for unit testing purposes.
 * See https://www.npmjs.com/package/apache-crypt
 * @kind function
 * @param {String} password - Password to hash.
 * @param {String} [encryptedPassword] - Contains salt. Salt will be randomly
 *   generated, if not provided.
 * @returns {String} - Hashed password.
 */
exports.crypt = require('apache-crypt');


/**
 * Performs a sha1 hash of the provided passsword.
 * @param {String} password - Password to hash.
 * @returns {String} - Hashed password.
 */
exports.sha1 = function(password) {
	let hash = crypto.createHash('sha1').update(password);
	return `{SHA}${hash.digest('base64')}`;
};
