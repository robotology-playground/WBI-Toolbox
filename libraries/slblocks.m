function blkStruct = slblocks

%  Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
%  Author: Jorhabib Eljaik Gomez
%  email: jorhabib.eljaik@iit.it
%  The development of this software was supported by the FP7 EU project
%  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
%  http://www.codyco.eu
%  Permission is granted to copy, distribute, and/or modify this program
%  under the terms of the GNU General Public License, version 2 or any
%  later version published by the Free Software Foundation.
%  
%  This program is distributed in the hope that it will be useful, but
%  WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
%  Author: Jorhabib Eljaik Gomez
%  email: jorhabib.eljaik@iit.it
%  
%  The development of this software was supported by the FP7 EU project
%  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
%  http://www.codyco.eu
%  
%  Permission is granted to copy, distribute, and/or modify this program
%  under the terms of the GNU General Public License, version 2 or any
%  later version published by the Free Software Foundation.
%  This program is distributed in the hope that it will be useful, but
%  WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details

Browser.Library   = 'WBCLibrary';     %Name of the .mdl file
Browser.Name      = 'Whole Body Interface Toolbox';    
Browser.IsFlat    =  0;

if (~verLessThan('matlab', '8.4'))  % R2014b
  % Add repository information if not yet done
  try
    load_system('WBCLibrary');
    if (strcmp(get_param('WBCLibrary', 'EnableLBRepository'), 'off'))
      set_param('WBCLibrary', 'Lock', 'off');
      set_param('WBCLibrary', 'EnableLBRepository', 'on');
      set_param('WBCLibrary', 'Lock', 'on');
      save_system('WBCLibrary');
    end;
    close_system('WBCLibrary');
  catch ex;
  end
end;


blkStruct.Browser =  Browser;
