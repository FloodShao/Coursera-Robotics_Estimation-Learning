I = imread('train/001.png');
clusters = 2;
pdf_test = zeros(size(I,1), size(I,2), clusters);
bw = zeros(size(I,1), size(I,2));
for n = 1:2
    for i = 1:size(I,1)
        for j = 1:size(I,2)
            pixel = [I(i,j,1) I(i,j,2) I(i,j,3)];
            hsv = rgb2hsv(double(pixel)/256);
            pdf_test(i,j,n) = gaussianND(hsv, mu(n,:), sigma{n});
            if pdf_test(i,j,n) > 0.8
                bw(i,j) = 1;
            end
        end
    end
end

bw_biggest = false(size(bw));

CC = bwconncomp(bw);
numPixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true;
figure,
imshow(bw_biggest);

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
