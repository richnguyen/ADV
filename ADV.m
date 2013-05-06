function ADV()
%% GLOBAL VARIABLES
global rrtS rrtG obs nTreeS nTreeG doPlot step buildingObs vehicleObs frame showTree;
showTree = 0;
doPlot = 0;
step = 0;
%% TUNING

BUILDING_DIM  = 150;
CAR_WIDTH  = 30;
CAR_LENGTH = 50;
CAR_SPEED = 7; % Speed of the vehicle; SPEED
MAX_STEER_ANGLE = pi/6; % 30 degree TURNING ANGLE
MIN_BOUNDARY = 25;  % How close to goal ? ACCURACY
K          = 2048; % RRTs have K vertices. CONSISTENCY
DELTA_T  = 8; % Inceasing at a rate of 16 interval FLEXIBILITY
ANIME_DELAY = 10; % SIMULATION DELAY

%% CONFIGURING
SCENE_DIM  = [700 800]; % scence dimension
BBOX = [0 800;0 700]; % Bounding (limit) of the scene
MAX_DISTANCE = 800;
N_MID_CONF  = 64; % Number of intermediate configuration in an edge
DO_NOT_PLOT = 0;
DO_PLOT = 0;
MAX_VERTEX = K+10; % maximum number of vertice a RRT has

B_HEIGHT = 90; % Button
B_WIDTH = 120;
LEFT_BOUND = 20;
ROW0= 50;
ROW1 = 150;
ROW2 = 250;
ROW3 = 350;
ROW4 = 450;
ROW5 = 550;
ROW6 = 650;
GAP = 10;
LINE_HEIGHT = 30;

%% CAR GRAPHICS
rawVerts = [0 0; ...
            0 CAR_WIDTH; ...
            CAR_LENGTH CAR_WIDTH; ...
            CAR_LENGTH 0; ...
            CAR_LENGTH-10 CAR_WIDTH; ...
            CAR_LENGTH-10  0 ];
originalVerts = rawVerts + repmat([-CAR_LENGTH/2, -CAR_WIDTH/2],6,1);
FACES = [1  2 3 4;...
    4 3 5 6];
CDATA = [0 .4 .2;...
    1 1 1];


%% BUILD FIGURE
% Define scene background
bg = ones(SCENE_DIM(1), SCENE_DIM(2));
% Define obstable image (for 2D scene)
obs = zeros(SCENE_DIM(1), SCENE_DIM(2));
close all; clc;
% Assign the GUI a name to  appear in the window title.
f = figure('Name','Automated Driving Vehicle (ADV)',...
    'Visible','off',...
    'Resize','off',...
    'Position',[360,500,SCENE_DIM(2)+LEFT_BOUND*3+B_WIDTH*2+GAP,SCENE_DIM(1)+LINE_HEIGHT*2+GAP*2]);
% Build viewport
frame = axes('Units','Pixels',...
    'Position',[LEFT_BOUND*2+B_WIDTH*2+GAP,ROW0,SCENE_DIM(2),SCENE_DIM(1)],...
    'GridLineStyle','--',...
    'XTickLabel','0|1',...
    'Color',[.9 .9 .9]);
imshow(bg); hold on;
% Build buttons


driveButton = uicontrol(f,'Style','pushbutton','FontSize',30,'String','Drive!',...
    'ForegroundColor',[0 .4 .2],...
    'Enable','inactive',...
    'Position', [LEFT_BOUND ROW3 B_WIDTH*2+GAP B_HEIGHT], 'Callback', @(src,event)driveCallBack(src,event));

driftButton = uicontrol(f,'Style','pushbutton','FontSize',30,'String','Drift',...
    'ForegroundColor','black',...
    'Enable','inactive',...
    'Position', [LEFT_BOUND ROW4 B_WIDTH*2+GAP B_HEIGHT], 'Callback', @(src,event)driftCallBack(src,event));

goalPosButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','Goal',...
    'ForegroundColor','red',...
    'Position', [LEFT_BOUND+B_WIDTH+GAP ROW5 B_WIDTH B_HEIGHT], 'Callback', @(src,event)goalPosCallBack(src,event));

startPosButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','Start',...
    'ForegroundColor',[0 .4 .2],...
    'Position', [LEFT_BOUND ROW5  B_WIDTH B_HEIGHT], 'Callback', @(src,event)startPosCallBack(src,event));

placeCarButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String',sprintf('Car'),...
    'ForegroundColor','blue',...
    'Position', [LEFT_BOUND+B_WIDTH+GAP ROW6  B_WIDTH B_HEIGHT], 'Callback', @(src,event)placeCarCallBack(src,event));

placeBuildingButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','Building',...
    'ForegroundColor','black',...
    'Position', [LEFT_BOUND ROW6  B_WIDTH B_HEIGHT], 'Callback', @(src,event)placeBuildingCallBack(src,event));

clearButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','Clear',...
    'Position', [LEFT_BOUND ROW2  B_WIDTH B_HEIGHT], 'Callback', @(src,event)clearCallBack(src,event));

reloadButton = uicontrol(f,'Style','pushbutton','FontSize',25,'String','Reload',...
    'ForegroundColor','black',...
    'Position', [LEFT_BOUND+B_WIDTH+GAP ROW2  B_WIDTH B_HEIGHT], 'Callback', @(src,event)reloadCallBack(src,event));

tuneButton = uicontrol(f,'Style','pushbutton','FontSize',30,'String','Tune',...
    'ForegroundColor','blue',...
    'Position', [LEFT_BOUND ROW1  B_WIDTH*2+GAP B_HEIGHT], 'Callback', @(src,event)tuneCallBack(src,event));

helpButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','Help',...
    'ForegroundColor','red',...
    'Position', [LEFT_BOUND ROW0  B_WIDTH B_HEIGHT], 'Callback', @(src,event)helpCallBack(src,event));

aboutButton = uicontrol(f,'Style','pushbutton','FontSize',24,'String','About',...
    'ForegroundColor','black',...
    'Position', [LEFT_BOUND+B_WIDTH+GAP ROW0  B_WIDTH B_HEIGHT], 'Callback', @(src,event)aboutCallBack(src,event));


feedback = uicontrol(f,'Style','edit','String','Demonstration of RRT Planner proposed by Steve LaValle',...
    'FontSize',15,...
    'ForegroundColor','white',...
    'BackgroundColor','black',...
    'Position',[LEFT_BOUND*2+B_WIDTH*2+GAP,ROW0-LINE_HEIGHT,SCENE_DIM(2),LINE_HEIGHT],'Visible','on');

instruction = uicontrol(f,'Style','edit','String','First, place some buildings in the scene',...
    'FontSize',15,...
    'ForegroundColor','black',...
    'BackgroundColor','white',...
    'Position',[LEFT_BOUND*2+B_WIDTH*2+GAP,ROW0+SCENE_DIM(1),SCENE_DIM(2),LINE_HEIGHT],'Visible','on');

%% Display figure

% Move the GUI to the center of the screen.
movegui(f,'center');
% Make the GUI visible.
set(f,'Visible','on');
set(f,'Toolbar','none','Menubar','none');


% Init source tree
nTreeS =0;
rrtS = initTree();
nTreeG = 0;
% Init goal tree
rrtG = initTree();
buildingObs = zeros(0,2);
vehicleObs  = zeros(0,2);

%% TUNER MENU
TEXT_WIDTH = 190;
NUM_WIDTH = 50;
MARGIN = 15;
S_WIDTH = 300;
S_HEIGHT  = 20;
COL1 = 20;
COL2 = COL1 + TEXT_WIDTH + MARGIN;
COL3 = COL2 + S_WIDTH  + MARGIN;
ROW1 = 20;
ROW2 = ROW1+2*(S_HEIGHT+MARGIN);
ROW3 = ROW1+3*(S_HEIGHT+MARGIN);
ROW4 = ROW1+4*(S_HEIGHT+MARGIN);
ROW5 = ROW1+5*(S_HEIGHT+MARGIN);
ROW6 = ROW1+6*(S_HEIGHT+MARGIN);
ROW7 = ROW1+7*(S_HEIGHT+MARGIN);
ROW8 = ROW1+8*(S_HEIGHT+MARGIN);
ROW9 = ROW1+9*(S_HEIGHT+MARGIN);
ROW10 = ROW1 + 10*(S_HEIGHT+MARGIN);
ROW11 = ROW1 + 11*(S_HEIGHT+MARGIN);
ROW12 = ROW1 + 12*(S_HEIGHT+MARGIN);

g = figure('Name','Tuning',...
    'Visible','off',...
    'Resize','off',...
    'Menubar','none',...
    'Position',[60,500,COL3+TEXT_WIDTH+MARGIN,ROW12+2*MARGIN]);
movegui(g,'center');

parentColor = get(g,'color');

tuneOkButton = uicontrol(g,'Style','pushbutton','FontSize',18,'String','Done',...
    'Position', [COL1 ROW1  TEXT_WIDTH S_HEIGHT*2], 'Callback', @(src,event)tuneOkCallBack(src,event));

tuneDefaultsButton = uicontrol(g,'Style','pushbutton','FontSize',18,'String','Defaults',...
    'Position', [COL2 ROW1  TEXT_WIDTH S_HEIGHT*2], 'Callback', @(src,event)tuneDefaultsCallBack(src,event));

tuneHelpButton = uicontrol(g,'Style','pushbutton','FontSize',18,'String','Help',...
    'Position', [COL2+TEXT_WIDTH+5 ROW1  TEXT_WIDTH S_HEIGHT*2], 'Callback', @(src,event)tuneHelpCallBack(src,event));


buildingText = uicontrol(g,'Style','text','String','Building Dimension',...
    'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW2, TEXT_WIDTH, S_HEIGHT]);
buildingSlider = uicontrol(g,'Style','slider',...
    'Max', 250 ,...
    'Min', 100,...
    'Value', BUILDING_DIM,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW2 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)buildingSliderCallback(src,event));
buildingValue = uicontrol(g,'Style','edit','String',num2str(BUILDING_DIM),...
    'Position',[COL3,ROW2,NUM_WIDTH,S_HEIGHT]);

carWidthText = uicontrol(g,'Style','text','String','Car Width',...
        'BackgroundColor',parentColor,...        
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW3,TEXT_WIDTH,S_HEIGHT]);
carWidthSlider = uicontrol(g,'Style','slider',...
    'Max', 40 ,...
    'Min', 10,...
    'Value', CAR_WIDTH ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW3 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)carWidthSliderCallback(src,event));
carWidthValue = uicontrol(g,'Style','edit','String',num2str(CAR_WIDTH),...
    'Position',[COL3,ROW3, NUM_WIDTH, S_HEIGHT]);

carLengthText = uicontrol(g,'Style','text','String','Car Length',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW4,TEXT_WIDTH,S_HEIGHT]);
carLengthSlider = uicontrol(g,'Style','slider',...
    'Max', 120 ,...
    'Min', 40,...
    'Value', CAR_LENGTH ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW4 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)carLengthSliderCallback(src,event));
carLengthValue = uicontrol(g,'Style','edit','String',num2str(CAR_LENGTH),...
    'Position',[COL3,ROW4, NUM_WIDTH, S_HEIGHT]);

carSpeedText = uicontrol(g,'Style','text','String','Car Speed',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW5,TEXT_WIDTH,S_HEIGHT]);
carSpeedSlider = uicontrol(g,'Style','slider',...
    'Max', 25 ,...
    'Min', 4,...
    'Value', CAR_SPEED ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW5 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)carSpeedSliderCallback(src,event));
carSpeedValue = uicontrol(g,'Style','edit','String',num2str(CAR_SPEED),...
    'Position',[COL3,ROW5, NUM_WIDTH, S_HEIGHT]);

steerAngleText = uicontrol(g,'Style','text','String','Steer Angle',...
        'BackgroundColor',parentColor,...        
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW6,TEXT_WIDTH,S_HEIGHT]);
steerAngleSlider = uicontrol(g,'Style','slider',...
    'Max', 60 ,...
    'Min', 15,...
    'Value', rad2deg(MAX_STEER_ANGLE) ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW6 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)steerAngleSliderCallback(src,event));
steerAngleValue = uicontrol(g,'Style','edit','String',num2str(rad2deg(MAX_STEER_ANGLE)),...
    'Position',[COL3,ROW6, NUM_WIDTH, S_HEIGHT]);

objectPropsText = uicontrol(g,'Style','text','String','Object Properties',...
        'BackgroundColor',parentColor,...
            'HorizontalAlignment','left',...
    'FontSize',16,'Position',[COL1,ROW7-5,S_WIDTH,S_HEIGHT+5]);

minBoundaryText = uicontrol(g,'Style','text','String','Accuracy',...
        'BackgroundColor',parentColor,...        
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW8,TEXT_WIDTH,S_HEIGHT]);
minBoundarySlider = uicontrol(g,'Style','slider',...
    'Max', 50 ,...
    'Min', 15,...
    'Value', MIN_BOUNDARY ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW8 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)minBoundarySliderCallback(src,event));
minBoundaryValue = uicontrol(g,'Style','edit','String',num2str(MIN_BOUNDARY),...
    'Position',[COL3,ROW8, NUM_WIDTH, S_HEIGHT]);

kText = uicontrol(g,'Style','text','String','Consistency',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW9,TEXT_WIDTH,S_HEIGHT]);
kSlider = uicontrol(g,'Style','slider',...
    'Max', 4096 ,...
    'Min', 512,...
    'Value', K ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW9 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)kSliderCallback(src,event));
kValue = uicontrol(g,'Style','edit','String',num2str(K),...
    'Position',[COL3,ROW9, NUM_WIDTH, S_HEIGHT]);

deltaTText = uicontrol(g,'Style','text','String','Flexibility',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW10,TEXT_WIDTH,S_HEIGHT]);
deltaTSlider = uicontrol(g,'Style','slider',...
    'Max', 32 ,...
    'Min', 4,...
    'Value', DELTA_T ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW10 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)deltaTSliderCallback(src,event));
deltaTValue = uicontrol(g,'Style','edit','String',num2str(DELTA_T),...
    'Position',[COL3,ROW10, NUM_WIDTH, S_HEIGHT]);

animeDelayText = uicontrol(g,'Style','text','String','Animation Delay',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','right',...
    'FontSize',12,'Position',[COL1,ROW11,TEXT_WIDTH,S_HEIGHT]);
animeDelaySlider = uicontrol(g,'Style','slider',...
    'Max', 20 ,...
    'Min', 2,...
    'Value', ANIME_DELAY ,...
    'SliderStep',[.1 .2],...
    'Position',[COL2 ROW11 S_WIDTH S_HEIGHT ],...
    'Callback',@(src,event)animeDelaySliderCallback(src,event));
animeDelayValue = uicontrol(g,'Style','edit','String',num2str(ANIME_DELAY),...
    'Position',[COL3,ROW11, NUM_WIDTH, S_HEIGHT]);

algorithmPropsText = uicontrol(g,'Style','text','String','Algorithm Properties',...
        'BackgroundColor',parentColor,...
    'HorizontalAlignment','left',...
    'FontSize',16,'Position',[COL1,ROW12-5,S_WIDTH,S_HEIGHT+5]);

showTreeCheckBox = uicontrol(g,'Style','checkbox','String','Show RRTs',...
        'BackgroundColor',parentColor,...
    'FontSize',12,'Position',[COL1+S_WIDTH, ROW12, TEXT_WIDTH, S_HEIGHT],...
    'Callback',@(src,event)showTreeCallback(src,event));

doStepCheckBox = uicontrol(g,'Style','checkbox','String','Step by Step',...
        'BackgroundColor',parentColor,...
    'FontSize',12,'Position',[COL1+2*TEXT_WIDTH+3*MARGIN, ROW12, S_WIDTH, S_HEIGHT],...
    'Callback',@(src,event)doStepCallback(src,event));

%% CALLBACK FUNCTIONS

    function placeBuildingCallBack(src,event)
        % place building in the enviroment
        xPos =0; yPos =0;
        while (xPos < BUILDING_DIM/2) || (xPos > SCENE_DIM(2)-BUILDING_DIM/2) || ...
                (yPos < BUILDING_DIM/2) || (yPos > SCENE_DIM(1)-BUILDING_DIM/2)
            [xPos yPos] = ginput(1);
            xPos = round(xPos);
            yPos = round(yPos);
        end
        draw_building([xPos yPos]);
        buildingObs = [buildingObs; [xPos yPos]];
        obs(yPos - BUILDING_DIM/2: yPos + BUILDING_DIM/2,...
            xPos - BUILDING_DIM/2: xPos + BUILDING_DIM/2) = 1;
        tell('Next, you can place more buildings or cars.');
    end

    function placeCarCallBack(src,event)
        % place car in the enviroment
        xPos = 0; yPos = 0;
        while (xPos < CAR_LENGTH/2) || (xPos > SCENE_DIM(2)-CAR_LENGTH/2) || ...
                (yPos < CAR_WIDTH/2) || (yPos > SCENE_DIM(1)-CAR_WIDTH/2)
            [xPos yPos] = ginput(1);
            xPos = round(xPos);
            yPos = round(yPos);
        end
        vehicleObs = [vehicleObs; [xPos yPos]];
        draw_car(xPos,yPos,0,'b');

        obs( yPos - CAR_WIDTH/2 : yPos + CAR_WIDTH/2,...
            xPos - CAR_LENGTH/2: xPos + CAR_LENGTH/2) = 1;
        tell('Good! Now you can place more buildings/cars or set a Start Location.');
    end

    function startPosCallBack(src,event)
        % set start (initial) position
        isCollide = 1; xPos =0; yPos =0;
        while (isCollide==1) || ...
                (xPos < CAR_LENGTH/2) || (xPos > SCENE_DIM(2)-CAR_LENGTH/2) || ...
                (yPos < CAR_WIDTH/2) || (yPos > SCENE_DIM(1)-CAR_WIDTH/2)
            [xPos yPos] = ginput(1);
            newPos = [round(xPos) round(yPos) 0];

            % check collision
            isCollide = check_collision(newPos,doPlot);
            talk('Place outside of obstacle','r');
        end
        draw_car(newPos(1),newPos(2),0,'r');
        talk('Start position acquired.','g');
        add_vertexS(newPos',0,0,[0 0]);
        set(startPosButton,'Enable','off');
        tell('Excellent! Now set a Goal Location.');
    end

    function goalPosCallBack(src,event)
        % set goal (finished) position
        isCollide = 1; xPos =0; yPos =0;
        while isCollide==1 || ...
                (xPos < CAR_LENGTH/2) || (xPos > SCENE_DIM(2)-CAR_LENGTH/2) || ...
                (yPos < CAR_WIDTH/2) || (yPos > SCENE_DIM(1)-CAR_WIDTH/2)
            [xPos yPos] = ginput(1);
            newPos = [round(xPos) round(yPos) 0];
            % check collision
            isCollide = check_collision(newPos,doPlot);
            talk('Place outside of obstacle','r');
        end
        draw_goal(newPos);


        talk('Goal position acquired.','g');
        add_vertexG(newPos', 0,0,[0 0]);
        set(goalPosButton,'Enable','off');
        tell('Perfect! Now click Drift or Drive to see how your car can automatically move.');
        set(driveButton,'Enable','on');
        set(driftButton,'Enable','on');

    end

    function clearCallBack(src,event)
        cla;
        bg = ones(SCENE_DIM(1), SCENE_DIM(2));
        frame = axes('Units','Pixels',...
            'Position',[LEFT_BOUND*2+B_WIDTH*2+GAP,ROW0,SCENE_DIM(2),SCENE_DIM(1)],...
            'GridLineStyle','--',...
            'XTickLabel','0|1',...
            'Color',[.9 .9 .9]);
        imshow(bg); hold on;
        % Clear obstacle image
        obs = zeros(SCENE_DIM(1), SCENE_DIM(2));
        % Clear RRTs
        nTreeS =0;
        rrtS = initTree();
        nTreeG = 0;
        rrtG = initTree();
        buildingObs = zeros(0,2);
        vehicleObs  = zeros(0,2);
        set_on_buttons();
        
        tell('Let''s place some buildings again.');
        talk('Demonstration of RRT Planner proposed by Steve LaValle.');
    end

    function reloadCallBack(src,event)
        cla;
        % compute and show RRT
        bg = ones(SCENE_DIM(1), SCENE_DIM(2));        
        frame = axes('Units','Pixels',...
            'Position',[LEFT_BOUND*2+B_WIDTH*2+GAP,ROW0,SCENE_DIM(2),SCENE_DIM(1)],...
            'GridLineStyle','--',...
            'XTickLabel','0|1',...
            'Color',[.9 .9 .9]);
        imshow(bg); hold on;
        % Clear obstacle image
        obs = zeros(SCENE_DIM(1), SCENE_DIM(2));
        % Clear RRTs
        nTreeS =0;
        rrtS = initTree();
        nTreeG = 0;
        rrtG = initTree();

        reload_obs();
        tell('Let''s set the Start and Goal positions.');
        set_on_buttons();
    end

    function driftCallBack(src,event)
        % compute and display the path which the car took from start to
        % goal
        %         talk('Thinking...','r');pause(0.01);
        build_rrt();
        % animation here
        tell('Here we go! You also can click Clear now to start over.');
    end

    function driveCallBack(src,event)
        % compute and display the path which the car took from start to
        % goal
        %         talk('Thinking...','r');pause(0.01);
        build_car_rrt();
        % animation here
        tell('Click Clear now to start over, or Reload to keep the Buildings/Cars');
    end

    function tuneCallBack(src, event)
        set(g,'Visible','on');
    end

    function aboutCallBack(scr,event)
        ab = figure('Name','About',...
            'Visible','off',...
            'Resize','off',...
            'Menubar','none',...
            'Position',[60,500,620,420]);
        movegui(ab,'center');
        aboutTxt = sprintf(['\n ADV: Automatic Driving Vehicle \n',...
            'Final project for Robot Motion Planning course \n',...
            'Computer Science Department \n The University of North Carolina at Charlotte \n\n',...
             '--------------------------------------------------------\n',...
            '\t Developed by Nhat ''Rich'' Nguyen \n', ...   
            '---------------------------------------------------------\n\n',...
            'Special thanks to Drs. Srinivas Akella and Steve LaValle. \n\n',...         
            '\t Thank you for using the program. \n',...
            '\t If you have any questions/comments, please contact: \n',...
            '\t \t rich.uncc@gmail.com \n\n',... 
            '----------------------------------------\n',...
            '\t\t May 2010 \n',...             
            ]);
        aboutText = uicontrol(ab,'Style','text','String',aboutTxt,...
            'FontSize',14,'Position',[10,10 ,600, 400]);
        
        set(ab,'Visible','on');
    end

    function helpCallBack(scr,event)
        ab = figure('Name','Help Guide',...
            'Visible','off',...
            'Resize','off',...
            'Menubar','none',...
            'Position',[60,500,620,420]);
        movegui(ab,'center');
        txt = sprintf(['Here is a quick instructions to use this program \n\n',...
            '\t 1. Click ''Building'' button to place some buildings in the viewport.\n',...
            '\t 2. Click ''Car'' button to place some cars in the viewport.\n',...
            '\t 3. Click ''Start'' button to set a starting position for your car.\n',...
            '\t 4. Click ''Goal'' button to set where you want to go.\n',...
            '\t 5. Click ''Drift'' button to execute walk-like motion planning OR\n',...
            '\t 6. Click ''Drive'' button to execute car-like motion.\n',...
            '\t 7. Click ''Clear'' button to clear everything in the viewport to start over. OR\n',...
            '\t 8. Click ''Reload'' button to keep the buildings and car.\n',...
            '\t 9. Repeat step 1. again. If you want to customize the settings of your car as well as the algorithm. Click ''Tune'' button. \n\n',...
            '------------------------------------------------------------------------\n',...
            'All documentations of the program is available at \n',...            
            '\t\t\t http://www.richnguyen.info/ADV.html',...
            ]);
        aboutText = uicontrol(ab,'Style','text','String',txt,...
            'HorizontalAlignment','left',...
            'FontSize',12,'Position',[10,10 ,600, 400]);
        
        set(ab,'Visible','on');
    end

    function buildingSliderCallback(scr,event)
        BUILDING_DIM = round(get(scr,'Value'));
        set(buildingValue,'String',num2str(BUILDING_DIM));
    end

    function carWidthSliderCallback(scr,event)
        CAR_WIDTH = round(get(scr,'Value'));
        set(carWidthValue,'String',num2str(CAR_WIDTH));
        rawVerts = [0 0; ...
            0 CAR_WIDTH; ...
            CAR_LENGTH CAR_WIDTH; ...
            CAR_LENGTH 0; ...
            CAR_LENGTH-10 CAR_WIDTH; ...
            CAR_LENGTH-10  0 ];
        originalVerts = rawVerts + repmat([-CAR_LENGTH/2, -CAR_WIDTH/2],6,1);

    end

    function carLengthSliderCallback(scr,event)
        CAR_LENGTH = round(get(scr,'Value'));
        set(carLengthValue,'String',num2str(CAR_LENGTH));
        rawVerts = [0 0; ...
            0 CAR_WIDTH; ...
            CAR_LENGTH CAR_WIDTH; ...
            CAR_LENGTH 0; ...
            CAR_LENGTH-10 CAR_WIDTH; ...
            CAR_LENGTH-10  0 ];
        originalVerts = rawVerts + repmat([-CAR_LENGTH/2, -CAR_WIDTH/2],6,1);

    end

    function carSpeedSliderCallback(scr,event)
        CAR_SPEED = round(get(scr,'Value'));
        set(carSpeedValue,'String',num2str(CAR_SPEED));
    end

    function steerAngleSliderCallback(scr,event)
        MAX_STEER_ANGLE = deg2rad(get(scr,'Value'));
        set(steerAngleValue,'String',num2str(round(get(scr,'Value'))));
    end

    function minBoundarySliderCallback(scr, event)
        MIN_BOUNDARY = round(get(scr,'Value'));
        set(minBoundaryValue,'String',num2str(MIN_BOUNDARY));
    end

    function kSliderCallback(scr, event)
        K = round(get(scr,'Value'));
        set(kValue,'String',num2str(K));
    end

    function deltaTSliderCallback(scr,event)
        DELTA_T =  round(get(scr,'Value'));
        set(deltaTValue,'String',num2str(DELTA_T));
    end

    function animeDelaySliderCallback(scr, event)
        ANIME_DELAY =  round(get(scr,'Value'));
        set(animeDelayValue,'String',num2str(ANIME_DELAY));
    end

    function showTreeCallback(scr,event)
        showTree = ~showTree;
    end

    function doStepCallback(scr,event)
       step = ~step; 
    end

    function tuneOkCallBack(scr, event)
        set(g,'Visible','off');
        set(0,'CurrentFigure',f);
    end

    function tuneDefaultsCallBack(scr, event)
        BUILDING_DIM  = 150;
        CAR_WIDTH  = 30;
        CAR_LENGTH = 50;
        CAR_SPEED = 7; % Speed of the vehicle; SPEED
        MAX_STEER_ANGLE = pi/6; % 30 degree TURNING ANGLE
        MIN_BOUNDARY = 25;  % How close to goal ? ACCURACY
        K          = 2048; % RRTs have K vertices. CONSISTENCY
        DELTA_T  = 8; % Inceasing at a rate of 16 interval FLEXIBILITY
        ANIME_DELAY = 10; % SIMULATION DELAY

        set(buildingValue,'String',num2str(BUILDING_DIM));
        set(carWidthValue,'String',num2str(CAR_WIDTH));
        set(carLengthValue,'String',num2str(CAR_LENGTH));
        set(carSpeedValue,'String', num2str(CAR_SPEED));
        set(steerAngleValue,'String', num2str(rad2deg(MAX_STEER_ANGLE)));
        set(minBoundaryValue,'String', num2str(MIN_BOUNDARY));
        set(kValue,'String',num2str(K));
        set(deltaTValue,'String',num2str(DELTA_T));
        set(animeDelayValue,'String',num2str(ANIME_DELAY));

        set(buildingSlider,'Value',BUILDING_DIM);
        set(carWidthSlider,'Value',CAR_WIDTH);
        set(carLengthSlider,'Value',CAR_LENGTH);
        set(carSpeedSlider,'Value',CAR_SPEED);
        set(steerAngleSlider,'Value',rad2deg(MAX_STEER_ANGLE));
        set(minBoundarySlider,'Value',MIN_BOUNDARY);
        set(kSlider,'Value',K);
        set(deltaTSlider,'Value',DELTA_T);
        set(animeDelaySlider,'Value',ANIME_DELAY);

    end

    function tuneHelpCallBack(scr,event)
       ab = figure('Name','Help Guide',...
            'Visible','off',...
            'Resize','off',...
            'Menubar','none',...
            'Position',[60,500,520,320]);
        movegui(ab,'center');
        txt = sprintf(['\t\t Need to write instruction  \n', ...
            '\t step 1: Place some buildings \n',...
            '\t step 2: Place some cars. \n\n',...
            'Refer to http://richnguyen.info/RMP for additional info.']);
        aboutText = uicontrol(ab,'Style','text','String',txt,...
            'FontSize',14,'Position',[10,10 ,500, 300]);
        
        set(ab,'Visible','on');
    end
%% UTILITY FUNCTIONS

%MASTER FUNCTION BUILD_RRT
    function build_rrt()
        % 1. Initialization
        rrtInit();
        swapped = 0;
        moveVector = [0,0]; % dummy variable for function       
        h = waitbar(0,'Computing route. Please wait...');        
        for it  = 1: K

            %             disp(['Iteration ' num2str(it)]);
            % 2. Build random configuration
            randConf = get_new_position();
            if doPlot==1, plot(randConf(1),randConf(2),'k*');end
            % 3. Find the nearest configuration
            [nearConfS nearIdS edge type]   = find_nearest_position(rrtS, randConf);
            if doPlot==1, plot(nearConfS(1),nearConfS(2),'ko');end
            if type == 2 % add the normal node in
                add_normal_vertexS(nearConfS', nearIdS);
            end
            % 4. Find the stopping configuration (in case of obstacles)

            stopConfS   = find_stopping_position(nearConfS, randConf);

            if is_the_same(stopConfS, nearConfS)==0
                % Add vertex to Source Tree
                if type == 1
                    add_vertexS(stopConfS', nearIdS, edge, moveVector);
                elseif type==2
                    add_vertexS(stopConfS', nTreeS, edge, moveVector);
                end

                [nearConfG nearIdG edge type] = find_nearest_position(rrtG, stopConfS);
                if doPlot==1, plot(nearConfG(1),nearConfG(2),'ko');end
                if type == 2 % add the normal node in
                    add_normal_vertexG(nearConfG', nearIdG);
                end


                stopConfG = find_stopping_position(nearConfG, stopConfS);


                if is_the_same(stopConfG ,nearConfG)==0
                    % Add vertex to goal tree
                    if type==1
                        add_vertexG(stopConfG', nearIdG, edge, moveVector);
                    elseif type==2
                        add_vertexG(stopConfG',nTreeG, edge, moveVector);
                    end
                end




                if is_the_same(stopConfG,stopConfS)==1
                    message = ['Solution found after ', num2str(it), ' iterations.'];
                    talk(message,'g');
                    % Make sure the tree are in the original order
                    if swapped == 1
                        swapTree();
                    end
                    close(h)
                    
                    set(0,'CurrentFigure',f);
                    path = trace_path();
                    if ~isempty(path)
                        if (showTree ==1)
                            show_tree(rrtS,'b');
                            show_tree(rrtG,'c');
                            plot(stopConfG(1),stopConfG(2),...
                                        'MarkerEdgeColor','r',...
                                        'MarkerFaceColor','y',...
                                        'MarkerSize',15);
                        elseif (step ==1)                            
                            step_by_step();
                            plot(stopConfG(1),stopConfG(2),...
                                        'MarkerEdgeColor','r',...
                                        'MarkerFaceColor','y',...
                                        'MarkerSize',15);
                        else
                            animate_path(path);
                        end
                        draw_path(path);
                    else
                        talk('Please RELOAD.','r');
                    end
                    break;
                    
                end

            end
            % Swap tree to obtain a balance search
            if is_bigger(rrtS, rrtG )==1
                swapTree();
                swapped = ~ swapped;
            end

            waitbar(it/K);

        end

        if it == K
            if step == 0
                close(h);
            end
            show_tree(rrtS,'b');
            show_tree(rrtG,'c');
            message = ['Cannot reach goal after ', num2str(it), ' iterations.'];
            talk(message,'r');
        end
        set_off_buttons();
    end

    function build_car_rrt()
        % 1. Initialization
        rrtInit();
        goalConf = rrtG(1).position;
        h = waitbar(0,'Computing route. Please wait...');
        for it  = 1: K
            %             disp(['Iteration ' num2str(it)]);
            % 2. Build random configuration
            randConf = get_new_position();
            if doPlot==1, plot(randConf(1),randConf(2),'k*');end
            % 3. Find the nearest configuration
            [nearConfS nearIdS edgeS]   = find_nearest_car_position(rrtS, randConf);
            if doPlot==1, plot(nearConfS(1),nearConfS(2),'ko');end
            % 4. Find the stopping configuration (in case of obstacles)
            [stopConfS moveVectorS]    = find_car_stopping_position (nearConfS, randConf);

            if is_the_same(stopConfS, nearConfS)==0
                % Add vertex to Source Tree
                add_vertexS(stopConfS', nearIdS, edgeS, moveVectorS);

                if is_within_boundary(stopConfS, goalConf)==1
                    message = ['Solution found after ', num2str(it), ' iterations.'];
                    talk(message,'g');
                    path = trace_car_path();
                    close(h);
                    set(0,'CurrentFigure',f);
                    if showTree ==1
                        show_car_tree(rrtS,'b',0);
                    elseif step == 1                       
                        show_car_tree(ANIME_DELAY);
                    else
                        animate_car_path(path);
                    end
                    draw_car_path(path);
                    break;
                end

            end
            waitbar(it/K);
        end

        if it == K
            close(h);
            show_car_tree(0);
            message = ['Cannot reach goal after ', num2str(it), ' iterations.'];
            talk(message,'r');
        end
        set_off_buttons();
    end

    function reload_obs()
        % Redraw building
        for b = 1: size(buildingObs,1)
            xPos = buildingObs(b,1);
            yPos = buildingObs(b,2);
            draw_building(buildingObs(b,:));
            obs(yPos - BUILDING_DIM/2: yPos + BUILDING_DIM/2,...
                xPos - BUILDING_DIM/2: xPos + BUILDING_DIM/2) = 1;
        end
        % Redraw cars
        for v = 1: size(vehicleObs,1)
            xPos = vehicleObs(v,1);
            yPos = vehicleObs(v,2);
            draw_car(xPos,yPos,0,'b');
            obs( yPos - CAR_WIDTH/2 : yPos + CAR_WIDTH/2,...
                xPos - CAR_LENGTH/2: xPos + CAR_LENGTH/2) = 1;
        end
    end

%% HELPER FUNCTIONS

    function newPos = get_new_position()
        % Get a random position within a certain range
        searchRange = abs(BBOX(:,1) - BBOX(:,2));
        isCollide = 1;
        while isCollide
            newPos = [round(searchRange(1)* rand(1)),...
                round(searchRange(2)* rand(1))];
            % check collision
            isCollide = check_collision(newPos,DO_NOT_PLOT);
        end
    end

    function [nearConf nodeId minDist] = find_nearest_car_position(rrt, randConf)
        % find the distance to nearest neighbors
        nearConf = rrt(1).position;
        nodeId  = 1;
        iTree = 2;
        active = rrt(iTree).active;
        minDist = get_distance(rrt(1).position, randConf);
        while active == 1 && iTree <= MAX_VERTEX
            distance = get_distance(rrt(iTree).position, randConf);
            if (distance < minDist)
                minDist = distance;
                nearConf = rrt(iTree).position;
                nodeId = iTree;
            end
            iTree = iTree + 1;
            active = rrt(iTree).active;
        end
    end

    function [nearConf nodeId minDist type] = find_nearest_position(rrt, randConf)
        % find the distance to nearest neighbors
        type = 1; % type = 1 : nearest vertex, type = 2: nearest edge
        nearConf = rrt(1).position;
        nodeId  = 1;
        iTree = 2;
        active = rrt(iTree).active;
        minDist = get_distance(rrt(1).position, randConf);
        while active == 1 && iTree <= MAX_VERTEX
            distance = get_distance(rrt(iTree).position, randConf);
            if (distance < minDist)
                minDist = distance;
                nearConf = rrt(iTree).position;
                nodeId = iTree;
                type = 1;
            end
            iTree = iTree + 1;
            active = rrt(iTree).active;
        end

        % distance to edge here
        iTree = 2; % do not count root node
        active = rrt(iTree).active;
        while active == 1 && iTree <= MAX_VERTEX
            nodeConf = rrt(iTree).position ;
            if (rrt(iTree).parent ~= 0) %fix bug for first node
                parentConf = [rrt(rrt(iTree).parent).position];
                [distance normConf] = get_distance_to_edge(nodeConf,parentConf,randConf);
                if distance < minDist
                    minDist = distance;
                    nearConf = [normConf rrt(iTree).orientation];
                    nodeId   = iTree;
                    type = 2;
                end
            end
            iTree = iTree  +1 ;
            active  = rrt(iTree).active;
        end
    end

    function stopConf  = find_stopping_position( nearConf, randConf)
        midConfArray = get_inter_points(nearConf, randConf, N_MID_CONF);
        n = 0;
        isCollide = 0;
        while (n < size(midConfArray,1)) && (isCollide == 0)
            n = n + 1;
            isCollide = check_collision(midConfArray(n,:),doPlot);
        end

        if n  == size(midConfArray,1)
            stopConf = randConf;
        elseif n >1
            stopConf = midConfArray(n-1,:);
        else
            stopConf = nearConf;
        end
    end

    function [stopConf moveVector]   = find_car_stopping_position (nearConf, randConf)
        minDistance = MAX_DISTANCE;
        stopConf = nearConf;
        moveVector = [0 0];

        % Try 6 different configuration
        gearVector = [-1 1];
        steerVector = [-1 0 1];
        for gear = 1: length(gearVector)
            for steer = 1: length(steerVector)
                speed = CAR_SPEED * gearVector(gear);
                steerAngle = MAX_STEER_ANGLE * steerVector(steer);
                % Update for delta_T time steps
                x = nearConf(1);
                y = nearConf(2);
                orientation = nearConf(3);

                xVector = zeros(DELTA_T+1,1);
                yVector = zeros(DELTA_T+1,1);
                xVector(1) = x;
                yVector(1) = y;

                for dt = 1: DELTA_T
                    [x, y, orientation] = position_update(x, y, orientation,...
                        speed, CAR_LENGTH , steerAngle);
                    xVector(dt+1) = x;
                    yVector(dt+1) = y;
                end
                if doPlot ==1, plot(xVector, yVector,'-k'); end
                isCollide = 0;
                dt = 1;
                while isCollide == 0 && dt < DELTA_T
                    isCollide = check_collision([xVector(dt), yVector(dt)],doPlot);
                    dt = dt + 1;
                end
                if isCollide==1
                    distance = MAX_DISTANCE;
                else
                    distance = norm(get_vector([x y], randConf));
                end

                if distance < minDistance
                    stopConf = [x y orientation];
                    minDistance = distance;
                    moveVector = [gearVector(gear) steerVector(steer)];
                end
            end
        end

        if doPlot==1, plot(stopConf(1), stopConf(2), '.m');end

    end

    function [x_new, y_new, orientation_new] = position_update(x, y, orientation,...
            speed, carLength, steerAngle)

        x_new = round( x + (speed * cos(orientation)));
        y_new = round( y + (speed * sin(orientation)));
        orientation_new = orientation + (speed/ carLength * tan(steerAngle));



    end


    function add_vertexS(conf,  parentId, edge, moveVector)
        nTreeS = nTreeS + 1;
        rrtS(nTreeS).active = 1;
        rrtS(nTreeS).position = conf';
        rrtS(nTreeS).parent = parentId;
        rrtS(nTreeS).edge = edge;
        rrtS(nTreeS).move = moveVector;
    end

    function add_vertexG(conf,  parentId, edge, moveVector)
        nTreeG = nTreeG + 1;
        rrtG(nTreeG).active = 1;
        rrtG(nTreeG).position = conf';
        rrtG(nTreeG).parent = parentId;
        rrtG(nTreeG).edge = edge;
        rrtG(nTreeG).move = moveVector;
    end

    function add_normal_vertexS(nearConfS,nearIdS)
        dis1 = get_distance(nearConfS, rrtS(nearIdS).position);
        dis2 = get_distance(nearConfS, rrtS(rrtS(nearIdS).parent).position);
        add_vertexS(nearConfS, rrtS(nearIdS).parent, dis2, [0 0]);
        % change structure
        rrtS(nearIdS).parent = nTreeS;
        rrtS(nearIdS).edge   = dis1;


    end

    function  add_normal_vertexG(nearConfG, nearIdG)
        dis1 = get_distance(nearConfG, rrtG(nearIdG).position);
        dis2 = get_distance(nearConfG, rrtG(rrtG(nearIdG).parent).position);
        add_vertexG(nearConfG, nearIdG, dis1,[0 0]);
        % change structure
        rrtG(rrtG(nearIdG).parent).parent = nTreeG;
        rrtG(rrtG(nearIdG).parent).edge   = dis2;
    end

    function path = trace_path()
        % Last on add in
        path = zeros(0,2);
        pathG = zeros(0,2);
        nodeG = rrtG(nTreeG);
        iter1 = 1;
        while (nodeG.parent ~= 0) && (iter1 < MAX_VERTEX)
            %            if size(nodeG.position,1)==2, nodeG.position = nodeG.position';end
            pathG = [pathG; nodeG.position(1:2)];
            % traverse tree
            nodeG = rrtG(nodeG.parent);
            iter1 = iter1 + 1;
        end
        % add Goal position
        pathG = [pathG; rrtG(1).position(1:2)];

        % Last on add in
        pathS = zeros(0,2);
        nodeS = rrtS(nTreeS);
        iter2 = 1;
        while (nodeS.parent ~= 0) && (iter2 < MAX_VERTEX)
            %             if size(nodeS.position,1)==2, nodeS.position = nodeS.position';end
            pathS = [pathS; nodeS.position(1:2)];
            % traverse tree
            nodeS = rrtS(nodeS.parent);
            iter2 = iter2  +1;
        end
        % add Start position
        pathS = [pathS; rrtS(1).position(1:2)];

        if (iter1 < MAX_VERTEX) && (iter2 < MAX_VERTEX)
            % Reverse order of pathS
            temp = zeros(size(pathS));
            for is = 1: size(pathS,1)
                temp(size(pathS,1) - is + 1,:) = pathS(is,:);
            end

            % Merge paths
            path = [temp(1:end-1,:); pathG(1:end,:)];
        end
    end

    function path = trace_car_path()
        % Last on add in
        pathS = zeros(0,5);
        nodeS = rrtS(nTreeS);
        iter = 1;
        while (nodeS.parent ~= 0) || (iter >= MAX_VERTEX)
            %             if size(nodeS.position,1)==2, nodeS.position = nodeS.position';end
            pathS = [pathS; nodeS.position nodeS.move];
            % traverse tree
            nodeS = rrtS(nodeS.parent);
            iter = iter  +1;
        end
        % add Start position
        pathS = [pathS; rrtS(1).position rrtS(1).move];

        % Reverse order of pathS
        temp = zeros(size(pathS));
        for is = 1: size(pathS,1)
            temp(size(pathS,1) - is + 1,:) = pathS(is,:);
        end

        % Merge paths
        path = temp;

    end

    function draw_car_path(path)
        % path: x,y,theta,gear,steer
        draw_car(path(1,1),path(1,2),0,'r');
        draw_goal(rrtG(1).position);
        for n = 2: size(path,1)
            speed = CAR_SPEED * path(n,4);
            steerAngle = MAX_STEER_ANGLE * path(n,5);
            % Update for delta_T time steps
            x = path(n-1,1);
            y = path(n-1,2);
            orientation = path(n-1,3);

            xVector = zeros(DELTA_T+1,1);
            yVector = zeros(DELTA_T+1,1);
            xVector(1) = x;
            yVector(1) = y;

            for dt = 1: DELTA_T
                [x, y, orientation] = position_update(x, y, orientation,...
                    speed, CAR_LENGTH , steerAngle);
                xVector(dt+1) = x;
                yVector(dt+1) = y;
            end
            plot(xVector, yVector,'-m','LineWidth',4,'Marker','.');

        end
        draw_car(path(n,1),path(n,2),path(n,3),'r');

    end


    function draw_path(path)
        hold on;
        for n = 1: size(path,1)-1
            line([path(n,1) path(n+1,1)],...
                [path(n,2) path(n+1,2)],...
                'Marker','.',...
                'LineWidth',4,...
                'Color','m',...
                'LineStyle','-');
        end
    end

    function animate_path(path)
        set(gcf,'DoubleBuffer','on') % To turn it on
        ANIME = struct('verts',{});
        counter = 1;   
        
        for n=1: size(path,1)-1
            midConfArray = get_inter_points(path(n,1:2),path(n+1,1:2), N_MID_CONF);
            for dt = 1:size(midConfArray,1)/16: size(midConfArray,1);
                x = midConfArray(dt,1);
                y = midConfArray(dt,2);
                counter = counter + 1;
                ANIME(counter).verts = repmat([x y],6,1) + (originalVerts);
            end

        end
         p = patch('Faces',FACES,'Vertices',ANIME(1).verts,...
                'FaceColor','flat',...
                'FaceVertexCData',CDATA,...
                'EdgeColor','k',...
                'EraseMode','normal',...
                'LineWidth',1);
        for a = 2: length(ANIME)
            set(p,'Vertices',ANIME(a).verts);
            drawnow
        end
        
%         draw_goal(rrtG(1).position);
%         draw_car(path(1,1),path(1,2),0,'r');
%         draw_car(path(end,1),path(end,2),0,'r');
    end

    function animate_car_path(path)
        set(gcf,'DoubleBuffer','on') % To turn it on        
        ANIME = struct('verts',{});
        counter = 1;        
                
        
        for n=2: size(path,1)
            speed = CAR_SPEED * path(n,4);
            steerAngle = MAX_STEER_ANGLE * path(n,5);
            % Update for delta_T time steps
            x = path(n-1,1);
            y = path(n-1,2);
            orientation = path(n-1,3);

            for dt = 1: DELTA_T
                [x, y, orientation] = position_update(x, y, orientation,...
                    speed, CAR_LENGTH , steerAngle);                
                % Clockwise rotation
                rotMatrix = [cos(orientation) sin(orientation);...
                -sin(orientation) cos(orientation)];
            
                counter = counter + 1;
                ANIME(counter).verts = repmat([x y],6,1) + (originalVerts *rotMatrix);
            end
        end
        draw_car(path(1,1),path(1,2),path(1,3),'w');        
        p = patch('Faces',FACES,'Vertices',ANIME(1).verts,...
                'FaceColor','flat',...
                'FaceVertexCData',CDATA,...
                'EdgeColor','k',...
                'EraseMode','normal',...
                'LineWidth',1);
        for a = 2: length(ANIME)
            set(p,'Vertices',ANIME(a).verts);
            drawnow
        end
        draw_car(path(1,1),path(1,2),path(1,3),'r');
    end

    function show_car_tree(delayTime)
       set(gcf,'DoubleBuffer','on') % To turn it on        
       ANIME = struct('verts',{});


        % exclude the root node, so start at 2
        for t = 2: nTreeS
            speed = CAR_SPEED * rrtS(t).move(1);
            steerAngle = MAX_STEER_ANGLE * rrtS(t).move(2);
            % Update for delta_T time steps
            x = rrtS(rrtS(t).parent).position(1);
            y = rrtS(rrtS(t).parent).position(2);
            orientation = rrtS(rrtS(t).parent).position(3);
            xVector = zeros(DELTA_T+1,1);
            yVector = zeros(DELTA_T+1,1);
            for dt = 1: DELTA_T
                xVector(dt) = x;
                yVector(dt) = y;
                [x, y, orientation] = position_update(x, y, orientation,...
                    speed, CAR_LENGTH , steerAngle);                  
            end           
            xVector(dt+1) = x;
            yVector(dt+1) = y;
            
            ANIME(t-1).xVector = xVector;
            ANIME(t-1).yVector = yVector;
        end
        deltaDelayTime = delayTime/nTreeS;
        
        for t = 2: nTreeS
            line(ANIME(t-1).xVector, ANIME(t-1).yVector,...
                'Color','b',...
                'LineStyle','-');
            pause(deltaDelayTime);
        end
    end

    function rrt = initTree()
        for i= 1: MAX_VERTEX
            rrt(i).active = 0;
            rrt(i).position = [0,0];
            rrt(i).orientation = 0;
            rrt(i).parent = 0;
            rrt(i).edge = 0;
            rrt(i).move = [0,0];
        end
    end


    function isCollide  = check_collision(position,doPlot)
        % Collision detection
        if (sum(isnan(position(1:2))) > 0) || (sum(position(1:2)==0) > 0) ||...
                (position(1) > SCENE_DIM(2)) || (position(2) > SCENE_DIM(1)) || ...
                (position(1) < 0 )   || (position(2) < 0)
            isCollide = 1;
            %             talk('Exception for location','r');
        else
            if obs(position(2), position(1)) ==1
                isCollide = 1;
                if doPlot==1,plot(position(1),position(2),'r.'),end
                %                 talk('Collision detected','red');
            else
                isCollide = 0;
                if doPlot==1,plot(position(1),position(2),'b.'),end
                %                 talk('Collision clear','green');

            end
        end
    end


    function rrtInit()

        get_c_free();
    end

    function get_c_free()
        % Dilate obs image to obtain C_FREE_SPACE
        CarRadius = round(sqrt(CAR_LENGTH^2 + CAR_WIDTH^2)/2)+1;
        se = strel('disk',CarRadius);
        %         figure(2), imshow(obs); title('Work Space');
        obs = round(imdilate(obs,se,2));
        %         figure(3), imshow(obs);title('Configuration Space');
        % Clear the border around the scence
        border = ones(size(obs));
        border(CarRadius : end - CarRadius , ...
            CarRadius : end - CarRadius) = zeros;

        obs = logical(obs) | logical(border);
        %         figure(2), imshow(obs);pause(); close;
    end



    function talk(message,color)
        if nargin==1, color = 'white'; end
        set(feedback,'String',message,'ForegroundColor',color);
    end

    function tell(message,color)
        if nargin==1, color = 'black'; end
        set(instruction,'String',message,'ForegroundColor',color);
    end

    function step_by_step()
        activeG = 1;
        activeS = 1;
        delayTime = ANIME_DELAY/max(nTreeS,nTreeG);
        t = 2;
        while (activeG ==1 ) || (activeS == 1)
            if (activeS ==1 )
            line( [rrtS(t).position(1) rrtS(rrtS(t).parent).position(1)],...
                [rrtS(t).position(2) rrtS(rrtS(t).parent).position(2)],...
                'Marker','.',...
                'Color','b',...
                'LineStyle','-');
            end
            if (activeG == 1)
            line( [rrtG(t).position(1) rrtG(rrtG(t).parent).position(1)],...
                [rrtG(t).position(2) rrtG(rrtG(t).parent).position(2)],...
                'Marker','.',...
                'Color','c',...
                'LineStyle','-');
            end
            t = t  +1;
            activeG = rrtG(t).active;
            activeS = rrtS(t).active;
            pause(delayTime);
        end
        
    end

    function show_tree(rrt,color)
        active = 1;
        t = 2; % exclude the root node
        while (active == 1) && (t < MAX_VERTEX)
            line( [rrt(t).position(1) rrt(rrt(t).parent).position(1)],...
                [rrt(t).position(2) rrt(rrt(t).parent).position(2)],...
                'Marker','.',...
                'Color',color,...
                'LineStyle','-');
            t = t  +1;
            active = rrt(t).active;
        end
    end

    

    function isSame = is_the_same(firstPos, secondPos)
        isSame = (firstPos(1)==secondPos(1)) & (firstPos(2) == secondPos(2));
    end

    function isSame = is_within_boundary(firstPos, secondPos)
        distance = norm(get_vector(firstPos, secondPos));
        if distance < MIN_BOUNDARY
            isSame = 1;
        else
            isSame = 0;
        end
    end

    function isBigger = is_bigger(rrtS, rrtG)
        if nTreeS > nTreeG
            isBigger = 1;
        else
            isBigger = 0;
        end
    end

    function swapTree()
        temp = rrtS;
        rrtS = rrtG;
        rrtG = temp;

        nTemp = nTreeS;
        nTreeS = nTreeG;
        nTreeG = nTemp;
    end

    function [distance, normConf]= get_distance_to_edge(nearConf,parentConf,randConf)
        % Make sure  the randConf stay within the line
        if (norm(get_vector(nearConf,parentConf)) > norm(get_vector(nearConf,randConf))) &&...
                (norm(get_vector(nearConf,parentConf)) > norm(get_vector(parentConf,randConf)))

            normConf = get_projection_point(nearConf, parentConf, randConf);
            distance = norm(get_vector(randConf, normConf));
        else
            distance = MAX_DISTANCE;
            normConf = [0 0];
        end
    end

    function v = get_vector(p1, p2)
        v= [(p2(1) - p1(1)) (p2(2) - p1(2)) ];
    end

    function unitVector = get_unit_vector(v)
        unitVector = v / norm(v);
    end

    function interPtArray = get_inter_points(p1 ,p2, nPt)
        v = get_vector(p1,p2);
        unitVector = get_unit_vector(v);
        vLength = norm(v)/nPt;
        interPtArray = zeros(nPt, 2);
        for ip = 1: nPt
            interConf = unitVector * (vLength * ip);
            % Change back to big coordinate by adding p1
            interPtArray(ip,:) = round(interConf) + p1(1:2);

        end
        %        plot(interPtArray(:,1),interPtArray(:,2),'xb');
    end

    function p = get_projection_point(p0, p1, q)
        A = [p1(1) - p0(1)  p1(2) - p0(2); ...
            p0(2) - p1(2)  p1(1) - p0(1)];

        b = -[ (-q(1))*(p1(1) - p0(1)) - q(2)*(p1(2) - p0(2)); ...
            (-p0(2))*(p1(1) - p0(1)) + p0(1)*(p1(2) - p0(2))];

        p = A\b;
        p = round(p');

    end

    function dist = get_distance(p1, p2)
        dist = sqrt(  (p2(1)- p1(1))^2 + (p2(2) - p1(2))^2 );
    end

    function draw_car(x,y,theta,color)    
        % Clockwise rotation
        rotMatrix = [cos(theta) sin(theta);...
             -sin(theta) cos(theta)];
        verts = repmat([x y],6,1) + (originalVerts *rotMatrix);
        if strcmp(color,'b')==1
            cdata = [0 0 1;...
                0 0 .5];
            patch('Faces',FACES,'Vertices',verts,...
                'FaceColor',color,...
                'FaceVertexCData',cdata,...
                'EdgeColor','k',...
                'LineWidth',1);
        elseif strcmp(color,'w')==1
            cdata = [1 1 1;...
                1 1 1];
            patch('Faces',FACES,'Vertices',verts,...
                'FaceColor','w',...
                'FaceVertexCData',cdata,...
                'EdgeColor','w',...
                'LineWidth',1);
        else
            cdata = [0 .4 .2;...
                1 1 1];
            patch('Faces',FACES,'Vertices',verts,...
                'FaceColor',color,...
                'FaceColor','flat',...
                'FaceVertexCData',cdata,...
                'EdgeColor','k',...
                'EraseMode','normal',...
                'LineWidth',1);
        end

    end

    function draw_goal(newPos)
        plot(newPos(1), newPos(2), 'o',...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','r',...
            'MarkerSize',CAR_LENGTH);
        plot(newPos(1), newPos(2), 'o',...
            'MarkerEdgeColor','w',...
            'MarkerFaceColor','w',...
            'MarkerSize',CAR_LENGTH-10);
        plot(newPos(1), newPos(2), 'o',...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','r',...
            'MarkerSize',25);
        plot(newPos(1), newPos(2), 'o',...
            'MarkerEdgeColor','w',...
            'MarkerFaceColor','w',...
            'MarkerSize',15);
        plot(newPos(1), newPos(2), 'o',...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','r',...
            'MarkerSize',5);
    end

    function draw_building(newPos)
        bRawVerts = [0 0; ...
            0 BUILDING_DIM; ...
            BUILDING_DIM BUILDING_DIM; ...
            BUILDING_DIM 0];

        bOriginalVerts = bRawVerts + repmat([-BUILDING_DIM/2, -BUILDING_DIM/2],4,1);
        bVerts = bOriginalVerts + repmat(newPos,4,1);
        bFaces = [1  2 3 4];
        patch('Faces',bFaces,'Vertices',bVerts,...
            'FaceColor','k',...
            'EdgeColor',[.7 .7 .7],...
            'LineWidth',5); hold on;

    end

    function radi = deg2rad(deg)
        radi = deg/180*pi;
    end

    function deg = rad2deg(radi)
        deg = round(radi/pi*180);
    end

    function set_off_buttons()
        set(driveButton,'Enable','off');
        set(driftButton,'Enable','off'); 
        set(placeBuildingButton,'Enable','off');
        set(placeCarButton,'Enable','off');
    end

    function set_on_buttons()
        set(startPosButton,'Enable','on');
        set(goalPosButton,'Enable','on');
        set(placeBuildingButton,'Enable','on');
        set(placeCarButton,'Enable','on');
        set(driveButton,'Enable','inactive');
        set(driftButton,'Enable','inactive'); 
    end

end
