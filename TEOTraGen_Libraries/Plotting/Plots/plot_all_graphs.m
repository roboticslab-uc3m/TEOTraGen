function plot_all_graphs(hObject,handles,S_V_A)

time = handles.result.trajectory.time;       
switch S_V_A
    
    case 'PLOT SPACE'
         
        q = handles.result.q;

        [m,n]=size(q);
        
        % Right Leg Joint Values
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj))))
            cla(handles.(strcat('axes',num2str(jj))),'reset')
            plot(time,q(jj,:));
            hold on
            title(handles.humanoid_structure.legs.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]')
            hold on
            plot(time,handles.humanoid_structure.legs.right.joint(jj).angle_limits(1,1)*ones(n,1),'r','LineWidth',2);
            plot(time,handles.humanoid_structure.legs.right.joint(jj).angle_limits(1,2)*ones(n,1),'r','LineWidth',2);
        end
        
        % Left Leg Joint Values
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+6))))
            cla(handles.(strcat('axes',num2str(jj+6))),'reset')
            plot(time,q(jj+6,:));
            hold on
            title(handles.humanoid_structure.legs.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]')
            hold on
            plot(time,handles.humanoid_structure.legs.left.joint(jj).angle_limits(1,1)*ones(n,1),'r','LineWidth',2);
            plot(time,handles.humanoid_structure.legs.left.joint(jj).angle_limits(1,2)*ones(n,1),'r','LineWidth',2);
        end
        
        % Waist Joint Values
        for jj=1:2
            axes(handles.(strcat('axes',num2str(jj+12))))
            cla(handles.(strcat('axes',num2str(jj+12))),'reset')
            plot(time,q(jj+12,:));
            hold on
            title(handles.humanoid_structure.waist.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]')
            hold on
            plot(time,handles.humanoid_structure.waist.joint(jj).angle_limits(1,1)*ones(n,1),'r','LineWidth',2);
            plot(time,handles.humanoid_structure.waist.joint(jj).angle_limits(1,2)*ones(n,1),'r','LineWidth',2);
        end
        
        
        % Right Arm Joint Values
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+14))))
            cla(handles.(strcat('axes',num2str(jj+14))),'reset')
            plot(time,q(jj+14,:));
            hold on
            title(handles.humanoid_structure.arms.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]')
            hold on
            plot(time,handles.humanoid_structure.arms.right.joint(jj).angle_limits(1,1)*ones(n,1),'r','LineWidth',2);
            plot(time,handles.humanoid_structure.arms.right.joint(jj).angle_limits(1,2)*ones(n,1),'r','LineWidth',2);
        end
        
        % Left Arm Joint Values
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+20))))
            cla(handles.(strcat('axes',num2str(jj+20))),'reset')
            plot(time,q(jj+20,:));
            hold on
            title(handles.humanoid_structure.arms.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]')
            hold on
            plot(time,handles.humanoid_structure.arms.left.joint(jj).angle_limits(1,1)*ones(n,1),'r','LineWidth',2);
            plot(time,handles.humanoid_structure.arms.left.joint(jj).angle_limits(1,2)*ones(n,1),'r','LineWidth',2);
          
        end
        
    case 'PLOT VELOCITIES'
        dq = handles.result.dq;
        [m,n]=size(dq);
        
        % Right Leg Velocities
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj))))
            cla(handles.(strcat('axes',num2str(jj))),'reset')
            plot(time,dq(jj,:));
            hold on
            title(handles.humanoid_structure.legs.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s')  
        end
        
        % Left Leg Velocities
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+6))))
            cla(handles.(strcat('axes',num2str(jj+6))),'reset')
            plot(time,dq(jj+6,:));
            hold on
            title(handles.humanoid_structure.legs.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s')   
        end
        
        % Waist Velocities
        for jj=1:2
            axes(handles.(strcat('axes',num2str(jj+12))))
            cla(handles.(strcat('axes',num2str(jj+12))),'reset')
            plot(time,dq(jj+12,:));
            hold on
            title(handles.humanoid_structure.waist.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s') 
        end
        
        % Right Arm Velocities
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+14))))
            cla(handles.(strcat('axes',num2str(jj+14))),'reset')
            plot(time,dq(jj+14,:));
            hold on
            title(handles.humanoid_structure.arms.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s')
        end
        
        % Left Arm Velocities
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+20))))
            cla(handles.(strcat('axes',num2str(jj+20))),'reset')
            plot(time,dq(jj+20,:));
            hold on
            title(handles.humanoid_structure.arms.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s')
        end
        
    case 'PLOT ACCELERATIONS'
        
        ddq = handles.result.ddq;       
        [m,n]=size(ddq);
        
        % Right Leg Acceleration
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj))))
            cla(handles.(strcat('axes',num2str(jj))),'reset')
            plot(time,ddq(jj,:));
            hold on
            title(handles.humanoid_structure.legs.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s^2')  
        end
        
        % Left Leg Acceleration
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+6))))
            cla(handles.(strcat('axes',num2str(jj+6))),'reset')
            plot(time,ddq(jj+6,:));
            hold on
            title(handles.humanoid_structure.legs.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s^2')      
        end
        
        % Waist Accelerations
        for jj =1:2
            axes(handles.(strcat('axes',num2str(jj+12))))
            cla(handles.(strcat('axes',num2str(jj+12))),'reset')
            plot(time,ddq(jj+12,:));
            hold on
            title(handles.humanoid_structure.legs.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s^2')      
        end        
        
        % Right Arm Accelerations
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+14))))
            cla(handles.(strcat('axes',num2str(jj+14))),'reset')
            plot(time,ddq(jj+14,:));
            hold on
            title(handles.humanoid_structure.arms.right.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s^2')
        end
        
        % Left Arm Accelerations
        for jj =1:6
            axes(handles.(strcat('axes',num2str(jj+20))))
            cla(handles.(strcat('axes',num2str(jj+20))),'reset')
            plot(time,ddq(jj+20,:));
            hold on
            title(handles.humanoid_structure.arms.left.joint(jj).name)
            xlabel('t [s]')
            ylabel('[rad]/s^2')
        end
        
        
end