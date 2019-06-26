function array = img(fname, varargin)
%this program is used to convert image file to C array
%para of scan sequency could be:
%  lrdu, left->right then down->up
%  lrud, left->right then up->down, normal sequency
%  rldu, right->left then down->up
%  rlud, right->left then up->down
%  dulr, down->up then left->right
%  udlr, up->down then left->right
%  durl, down->up then right->left
%  udrl, up->down then right->left
%BGR565 format, 16bit per pixel

scan = 'dulr';
if nargin == 2,
    scan = varargin{1};
end

diary(sprintf('%s.c', fname));
diary on
array = imread(fname);
[h, w, colors] = size(array);

for x = 1:h,
    for y = 1:w,
        r = uint16(array(x,y,1));
        g = uint16(array(x,y,2));
        b = uint16(array(x,y,3));
        r = bitshift(r, -3);
        g = bitshift(g, -2);
        b = bitshift(b, -3);
        rgb(x,y) = bitshift(b, 11) + bitshift(g, 5) + r;
    end
end

array = rgb;

% note:
% matlab default array scan sequency is udlr
switch scan
    case 'dulr',
        array = flipud(array);
    case 'lrud',
        array = flipud(rot90(array));
    otherwise,
end

%disp head
disp('/*****************************************************');
disp('* image file generated by img.m v1.0');
str = sprintf('* BGR565 format - %s', fname);
disp(str);
disp('* copyright (c) miaofng@2012');
disp('******************************************************/');

str = sprintf('const char img_%s[] = {', strrep(fname, '.', '_'));
disp(str)
str = sprintf('\t0x%04x, 0x%04x, /*width = %d, height = %d, scan = %s*/', w, h, w, h, scan);
disp(str)

n = 0;
N = h * w;
while n < N
    str = sprintf('\t');
    I = min(N - n, 8);
    for i = 1:I
        str = [str, sprintf('0x%04x, ', array(n + i))];
    end
    str = [str, ' \'];
    disp(str);
    n = n + I;
end

disp('};')
diary off

fid = fopen(sprintf('%s.bin', fname), 'wb');
fwrite(fid, array, 'uint16');
fclose(fid);
