function PrintLinkName(j)
global uLINK  % Refere global variable

if j ~= 0
    fprintf('j=%2d : %s\n',j,uLINK(j).name); % Show my name
    PrintLinkName(uLINK(j).child);       % Show my child's name
    PrintLinkName(uLINK(j).sister);      % Show my sister's name
end
