function testcodeML()
    disp('Program started');
    sim=remApi('remoteApi');
    sim.simxFinish(-1);
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    %client.simxAddStatusbarMessage('Hello world!',client.simxDefaultPublisher());
    %sim.simxAddStatusbarMessage(clientID,'Started simulation',2);
    if (clientID>-1)
        disp('Connected to remote API server');
        sim.simxSynchronous(clientID,true);
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
        
        %Max & min speed
        Speed=[15*pi/180 90*pi/180];
        % Handle
        [~,motorLeft]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
        [~,motorRight]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
        [~,goal]=sim.simxGetObjectHandle(clientID,'goal',sim.simx_opmode_blocking);
        [~,obj]=sim.simxGetObjectHandle(clientID,'position',sim.simx_opmode_blocking);
        sim.simxSetObjectParent(clientID, goal, -1, true, 2);
        
        
        [~,camera]=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);
        
        %Job Code
        [~,~,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_streaming);
        [~,sp]=sim.simxGetObjectPosition(clientID, goal, obj, 2);
        %Окно ввода координат цели
        prompt1 = {'Введите координаты цели по x (-4.5 4.5): ', 'Введите координаты цели по y(-4.5 4.5): '};
        title = 'Ввод новых координат цели';
        target = cellfun(@str2double,inputdlg(prompt1,title));
        sim.simxSetObjectPosition(clientID, goal,-1,[target(1),target(2),1.3865e-01],sim.simx_opmode_blocking);
        
        for i=1:50
            [~,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_buffer);
            [~,sp]=sim.simxGetObjectPosition(clientID, goal, obj, 2);
            angleB = atan(sp(2)/sp(1));
            angleBdeg = rad2deg(angleB);
            out = imshow(image);
            
        d=5;    
        %Условия начала движения
        if ((abs(sp(2))>0.1) | (abs(sp(1))>0.1))
            %Определение 1 и 4 четвертей
            if (sp(1)>=0 & (abs(angleBdeg)>=0 & abs(angleBdeg)<=90))
                %Поворачиваем налево
                if (angleBdeg>d & abs(angleBdeg)>(d*2))
                    vL=-Speed(1);
                    vR=Speed(1);
                %Поворачиваем направо
                elseif (angleBdeg<-d & abs(angleBdeg)>(d*2))
                    vL=Speed(1);
                    vR=-Speed(1);
                %Едем прямо
                else
                    vL=Speed(2);
                    vR=Speed(2);
                end
            %Определение 2 и 3 четвертей
            elseif (sp(1)<0 & sp(2)~=0  & (angleBdeg>1 | angleBdeg<-1))
                %Поворачиваем направо
                if (angleBdeg>d & abs(angleBdeg)>(d*2))
                    vR=-Speed(1);
                    vL=Speed(1);
                %Поворачиваем налево
                elseif (angleBdeg<-d & abs(angleBdeg)>(d*2))
                    vL=-Speed(1);
                    vR=Speed(1);
                end
            %Цель позади, разворот
            elseif (sp(1)<0 & (angleBdeg<1 | angleBdeg>-1))
                    vR=-Speed(1);
                    vL=Speed(1);         
            end
        %Остановка по достижении цели
        elseif ((abs(sp(2))<0.1) & (abs(sp(1))<0.1))
            vR=0;
            vL=0;         
        end
        
        sim.simxSetJointTargetVelocity(clientID,motorLeft,vL,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID,motorRight,vR,sim.simx_opmode_blocking);

        sim.simxSynchronousTrigger(clientID);
        end    
    
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
    
    else
        disp('Failed connecting to remote API server');
    end
    close(out.Parent.Parent);

    sim.delete(); 
    disp('Program end')
end