function hzk(fname)
%this program is used to convert binary font file generated by 
%HZKCreator.exe to C array
%when font 16 is choosed, binary file is following for only 1
%ascii 'A' 0x41:
%0000000 0000 1000 1810 2828 3c24 4244 e742 0000
%0000010 0808 0000 4241
%0808 width and height, there are something wrong here, in fact
%     height is always two times of width
%4241 ascii code from 0x41 to 0x42(not included)

fid = fopen(fname, 'rb');
array = fread(fid, inf, 'uint8');
fclose(fid);

len = length(array);
code = array(len - 1); %include
n_chars = array(len) - code; %total nr of chars
bytes = len - 6;
bytes_per_char = bytes / n_chars;


N = len; %bytes
I = 8; %bytes per line to display

%disp head
disp('/*****************************************************');
disp('* font file generated by HZKCreator v1.0');
disp('* font file generated by hzk.m v1.0');
disp('*');
disp('* copyright (c) miaofng@2010');
disp('******************************************************/');

w = array(len - 5);
h = w * 2;
str = sprintf('const char ascii_%dx%d[] = {', w, h);
disp(str)

n = 0;
m = bytes_per_char;
while n < N
    if m == bytes_per_char
        %disp char info
        str = sprintf('	/* "%c" - 0x%02x */', code, code);
        disp(str)
        code = code + 1;
        m = 0;
    end

    i = 0;
	str='	';
	while n < N
		val = array(n + 1);
		if val < 0 %complement code
			val = bitset(bitxor(abs(val), (2^8-1)), 8)+1;
		end
		dat = sprintf('0x%02x, ',val);
		str = [str, dat];
        
        m = m + 1;
        n = n + 1;
        i = i + 1;
        
        if i >= I %over display bytes per line limit
            break;
        end
        
        if m == bytes_per_char %over bytes of a char limit
            break;
        end
	end
	str=[str, '\'];
	disp(str)
end
disp('};')