## Furuta Pendulum

Lydia Altman, Cyril Durovchic, Jessica Oliveira, Nathaniel Warfield, Christopher Warnberg

**Introduction**


The goal of this project was to model the physical system and design the control system for a rotary inverted pendulum. There are two primary functions of the rotary inverted pendulum and they are to firstly invert the pendulum and then to maintain its unstable inverted position. This is done by applying a torque to the arm of the pendulum. The governing equations of the system were written then linearized and input into Matlab Simulink. A model of the physical system was created in CoppeliaSim. This report contains the methodology used to derive the models.


You can use the [editor on GitHub](https://github.com/Fall-2020-MECA-482-Furuta-Pendulum/jtracy-oliveira.github.io/edit/gh-pages/index.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

MATLAB Code:
```markdown
%Furuta Pendulum Simulation Code
%State-Space Representation
%%Assign Arbitrary Values to Appropriate Variables
%%Variable Definitions
%   Io: inertia of arm
%   Lo: length of arm
%   m1: mass of pendulum
%   l1: distance to c.g. of pendulum
%   J1: inertia around c.g. of pendulum
%   thetao: rotational angle of arm
%   theta1: rotational angle of pendulum
%   tau: input torque to arm
%   g: gravitational acceleration

disp('Commence Program');
sim=remApi('API'); 
sim.simxFinish(-1); 
clientID=sim.simxStart('12/20/20',999,true,true,1000,2);

Io = .25;
Lo = .5;
m1 = 2.5;
l1 = .25;
J1 = .25;
g = 9.81;

%   A: matrix A
%   B: matrix B
%   A23: equation at 2,3
%   A43: equation at 4,3
%   B21: equation at 2,1
%   B41: equation at 4,1

A23 = -m1*l1*Lo*g/(Io*(J1+m1*l1^2)+J1*m1*Lo^2);
A43 = (Io+m1*Lo^2)*m1*l1*g/(Io*(J1+m1*l1^2)+J1*m1*Lo^2);
B21 = (J1+m1*l1^2)/(Io*(J1+m1*l1^2)+J1*m1*Lo^2);
B41 = -m1*l1*Lo/(Io*(J1+m1*l1^2)+J1*m1*Lo^2);

   if (clientID>0)
A = [0 1 0 0; 0 0 A23 0; 0 0 0 1; 0 0 A43 0];
B = [0; B21; 0; B41];
I = eye(4);
s = sym('s');
d = det(s*I-A);
eigs = [sqrt(20.809); 0; -sqrt(20.809); 0]
K = place(A,B,eigs);

disp('API Server Connection Established');
    [returnCode]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
    [returnCode,PendulumBrg]=sim.simxGetObjectHandle(clientID,'Pendulum Axis',sim.simx_opmode_blocking);
    [returnCode,Pendulum]=sim.simxGetObjectHandle(clientID,'Cyl 2',sim.simx_opmode_blocking);
    [returnCode,Arm]=sim.simxGetObjectHandle(clientID,'Cyl 1',sim.simx_opmode_blocking);
    [returnCode,ArmBrg]=sim.simxGetObjectHandle(clientID,'Motor Axis',sim.simx_opmode_blocking);
    
    t=clock;
    StartingTime=t(1);
    CurrentTime=t(1);

    [returnCode,thetao]=sim.simxGetJointPosition(clientID,Arm,sim.simx_opmode_streaming);
    [returnCode,LinearVelArm,AngularVelArm]=sim.simxGetObjectVelocity(clientID,Arm,sim.simx_opmode_streaming);
    [returnCode,theta1]=sim.simxGetJointPosition(clientID,PendulumBrg,sim.simx_opmode_streaming);
    [returnCode,LinearVelPendulum,AngularVelPendulum]=sim.simxGetObjectVelocity(clientID,Pendulum,sim.simx_opmode_streaming);
    

    i = 1;
    while (CurrentTime-StartingTime < 10) 
        [returnCode,thetao]=sim.simxGetJointPosition(clientID,ArmBrg,sim.simx_opmode_streaming);
        [returnCode,LinearVelArm,AngularVelArm]=sim.simxGetObjectVelocity(clientID,Arm,sim.simx_opmode_streaming);
        [returnCode,theta1]=sim.simxGetJointPosition(clientID,PendulumBrg,sim.simx_opmode_streaming);
        [returnCode,LinearVelPendulum,AngularVelPendulum]=sim.simxGetObjectVelocity(clientID,Pendulum,sim.simx_opmode_streaming);
        if (returnCode==sim.simx_return_ok)
            
                TimeArray(i,1) = CurrentTime;
                TimeArray(i,2) = thetao;
                TimeArray(i,3) = AngularVelArm;
                TimeArray(i,4) = theta1;
                TimeArray(i,5) = atan(tan(-AngularVelPendulum(1))/cos(thetao));
                TimeArray(i,6) = tau;
                thetadot = -atan(tan(-AngularVelPendulum(1))/cos(thetao);
                [returnCode]=sim.simxSetJointForce(clientID,ArmBrg,tau,sim.simx_opmode_oneshot);

                if theta1>.001 && theta1<.99*pi()
                    tau = 2*K*[pi()-thetao;AngularVelArm(3);theta1;thetadot];
                    [returnCode]=sim.simxSetJointTargetVelocity(clientID,ArmBrg,-5,sim.simx_opmode_streaming);
                elseif theta1>-.99*pi() && theta1<-.049
                    tau = 2*K*[pi()-thetao;AngularVelArm(3);theta1;thetadot];
                    [returnCode]=sim.simxSetJointTargetVelocity(clientID,ArmBrg,5,sim.simx_opmode_streaming);
                else
                    tau = 0;
                end
 
                i = i+1;
                
        end
        
        t=clock;
        CurrentTime=t(1);
    end

else
        disp('Could Not Establish Server Connection');
end

disp('Program Completed');
plot(timeMatrix(:,1), timeMatrix(:,2))
plot(timeMatrix(:,1), timeMatrix(:,3))
plot(timeMatrix(:,1), timeMatrix(:,4))
plot(timeMatrix(:,1), timeMatrix(:,5))
plot(timeMatrix(:,1), timeMatrix(:,6))
legend('thetao','AngularVelocityArm','theta1','ArmPosition','tau')

```
# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text
!.[Furuta Pendulum 1](Furuta Pendulum 1.png)
[Link](url) and ![Image](src)
For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/Fall-2020-MECA-482-Furuta-Pendulum/jtracy-oliveira.github.io/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://docs.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
