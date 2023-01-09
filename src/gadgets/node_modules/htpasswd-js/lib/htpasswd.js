const checkUtils = require('./check-utils');

/**
 * Represents parsed htpasswd data.
 * @private
 * @param {Object} [hashes={}] - Map from usernames to password hashes.
 */
class Htpasswd {
	constructor(hashes = {}) {
		this.hashes = hashes;
	}

	/**
	 * Parses an htpasswd data string.
	 * @static
	 * @param {String} str - String containing usernames and password hashes in
	 *   the htpasswd format.
	 * @returns {Htpasswd} - Parsed instance.
	 */
	static parse(str) {
		let htpasswd = new Htpasswd();
		let lines = str.split(/\r?\n/);
		for (let line of lines) {
			let [ username, hash ] = line.split(/:(.*)/);
			if (hash) htpasswd.hashes[username] = hash;
		}
		return htpasswd;
	}

	/**
	 * Gets the password hash for the provided username, if any.
	 * @param {String} username - Username which may or may not exist.
	 * @returns {String|null} - Password hash associated with the username, or
	 *   null if no such user exists.
	 */
	getHash(username) {
		return this.hashes[username] || null;
	}

	/**
	 * Checks if the provided username and password combination is valid.
	 * @param {String} username - Username which may or may not exist.
	 * @param {password} password - Password which may or may not be correct.
	 * @returns {Promise<boolean>} - Will resolve with true if the combination
	 *   is valid, false otherwise.
	 */
	authenticate(username, password) {
		let hash = this.getHash(username);
		if (!hash) return Promise.resolve(false);
		return checkUtils.checkPassword(password, hash);
	}
}

module.exports = Htpasswd;
