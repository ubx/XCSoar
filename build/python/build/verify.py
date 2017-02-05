import hashlib

def feed_file(h, f):
    """Feed data read from an open file into the hashlib instance."""

    while True:
        data = f.read(65536)
        if len(data) == 0:
            # end of file
            break
        h.update(data)

def feed_file_path(h, path):
    """Feed data read from a file (to be opened by this function) into the hashlib instance."""

    with open(path, 'rb') as f:
        feed_file(h, f)

def file_digest(algorithm, path):
    """Calculate the digest of a file and return it in hexadecimal notation."""

    h = algorithm()
    feed_file_path(h, path)
    return h.hexdigest()

def file_md5(path):
    """Calculate the MD5 checksum of a file and return it in hexadecimal notation."""

    return file_digest(hashlib.md5, path)

def verify_file_digest(path, expected_digest):
    """Verify the digest of a file, and return True if the digest matches with the given expected digest."""

    return file_md5(path) == expected_digest