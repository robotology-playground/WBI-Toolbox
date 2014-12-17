% Assuming 'model.urdf' is in $CODYCO_SUPERBUILD_ROOT/libraries/yarpWholeBodyInterface/app/robots/$YARP_ROBOT_NAME
function linksNames =  parseLinksNamesFromURDF(filename)
try
   urdf = xmlread(filename);
catch
   error('Failed to read XML file %s.',filename);
end

%% Get all link names in the urdf file
links = urdf.getElementsByTagName('link');
linksNames = cell(links.getLength, 1);
for i=0:links.getLength - 1
    link = links.item(i);
    linksNames{i+1} = char(link.getAttribute('name'));
%     disp(linksNames{i+1});
end

end