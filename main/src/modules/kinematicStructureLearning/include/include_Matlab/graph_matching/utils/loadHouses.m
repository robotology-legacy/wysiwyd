function houses = loadHouses()

for i = 1:111
    fname = ['data/cmum/house/label/house', num2str(i)];
    f = fopen(fname);
    houses{i} = fscanf(f, '%f', [2 Inf]);
    fclose(f);
end
