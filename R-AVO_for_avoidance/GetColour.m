function [Colour] = GetColour(PursuerNo)
    % GetColour - Returns the RGB colour matrix for the given pursuer
    % 0 - for Target colour, and the corresponding integer to get the corresponding pursuer

    %ColourHexList_string = ['e41a1c', '377eb8', '4daf4a', '984ea3', 'ff7f00', 'a65628', 'ffff33', 'f781bf', '999999'];
    ColourHexList_string = ['999999','e41a1c', '377eb8', '4daf4a', 'ffa600', '984ea3', '4daf4a', 'a65628', 'f781bf']; %https://colorbrewer2.org/#type=qualitative&scheme=Set1&n=5
    for iCount=1:1:int16(size(ColourHexList_string,2)/6)
        ColourRGBList(iCount,:) = sscanf(ColourHexList_string((iCount-1)*6 + 1:iCount*6),'%2x%2x%2x',[1 3])/255;
    end

    Colour = ColourRGBList(PursuerNo+1,:);

end
