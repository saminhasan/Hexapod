function hash = sha256(file)
    % SHA-256 of a file's bytes as a lowercase hex string
    fid = fopen(file, 'r');
    bytes = fread(fid, inf, '*uint8');
    fclose(fid);
    md = java.security.MessageDigest.getInstance('SHA-256');
    hash = lower(reshape(dec2hex(typecast(md.digest(typecast(bytes, 'int8')), 'uint8'))', 1, []));
end
