# htpasswd-js

Pure JS htpasswd authentication.

This is based on [htpasswd-auth](https://www.npmjs.com/package/htpasswd-auth),
except that it uses [bcryptjs](https://www.npmjs.com/package/bcryptjs) for
bcrypt hashes. This module does not depend on any C++ node add-ons, making it
more portable. It is inevitably slower, however.


## Usage
Simply provide the necessary arguments to the `::authenticate` method:

```js
const htpasswd = require('htpasswd-js');

htpasswd.authenticate({
	username: 'username',
	password: 'password',
	file: '/absolute/path/to/file.htpasswd'
})
	.then((result) => {
		// Result will be true if and only if
		// username and password were correct.
	});
```

It is also possible to provide htpasswd data directly, instead of reading it
from a file:

```js
const htpasswd = require('htpasswd-js');

htpasswd.authenticate({
	username: 'username',
	password: 'password',
	data: 'username: {SHA}W6ph5Mm5Pz8GgiULbPgzG37mj9g=\n'
})
	.then((result) => {
		// Result will be true if and only if
		// username and password were correct.
	});
```


## Note on Asynchronous Authentication

It may not seem like the `authenticate` method needs to be asynchronous,
especially when the data is provided directly. No disk operations or
interactions with other processes are necessary that case.

The reason why it is asynchronous, however, is because the `bcryptjs` module
allows it:

> Note: Under the hood, asynchronisation splits a crypto operation into small
> chunks. After the completion of a chunk, the execution of the next chunk is
> placed on the back of JS event loop queue, thus efficiently sharing the
> computational resources with the other operations in the queue.

This is the most desirable behavior, because you may need to consume high-cost
bcrypt hashes. Synchronous operation would be possible, but it is rarely a good
idea to block execution for potentially long periods of time.


## Plain Text Passwords

Unlike `htpasswd-auth` and other similar modules, plain text passwords are
*not* supported by `htpasswd-js`. The reason for this is that there's no real
way in the htpasswd format of telling the difference between a `crypt(3)` hash
and plain text. This is because the plain text and `crypt(3)` passwords are
never supported by the same Apache instance. You get one or the other, based on
which operating system you're running on.

Some other modules will check the unhashed password for equality with the hash
before checking it with `crypt(3)`. This is a bad idea, because it potentially
allows an attacker to enter `crypt(3)` hashes directly, without even needing to
do cryptanalysis to obtain the password.

Also, you *really* shouldn't be storing passwords as plain text. Or `crypt(3)`.
Or really anything but `bcrypt` at this point. Why? See below.


## Please Use `bcrypt`

`bcrypt` is the only htpasswd-supported password format that still holds up to
modern cryptanalysis.

- `MD5` was declared
  [cryptographically broken](https://www.kb.cert.org/vuls/id/836068) in 2008.
  Attacks against it can run on an average computer in less then a second.
- `SHA1` has its own vulnerabilities, and the algorithm used by htpasswd does
  not use a random salt, making it additionally vulnerable to lookup tables and
  rainbow tables.
- `crypt(3)`'s traditional DES-based algorithm, which is used by htpasswd, is
  very old and does not hold up to dictionary attacks on modern hardware.
- Plain text of course doesn't require any cryptanalysis because it's plain
  text.

I supported the others for sake of completeness and also because you won't
always be able to control which formats are used in your organization. If you
can, however, make sure to use `bcrypt` and only `bcrypt`.


## Editing Files

`htpasswd-js` only supports authentication. To actually edit htpasswd files,
you may want to use Apache's htpasswd utility itself, or its
[node port](https://www.npmjs.com/package/htpasswd). Fortunately, the node
port also uses `bcryptjs`, so it doesn't need any C++ add-ons, either.
