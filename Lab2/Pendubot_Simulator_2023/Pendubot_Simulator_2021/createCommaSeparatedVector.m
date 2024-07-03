function resultVector = createCommaSeparatedVector(inputData)
    % Convert numeric array to string
    inputDataStr = num2str(inputData);
    
    % Split the string by spaces (since num2str separates numbers by spaces)
    lines = strsplit(inputDataStr);

    % Join with commas
    resultVector = strjoin(lines, ', ');
end

