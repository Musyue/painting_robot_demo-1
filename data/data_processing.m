clc,clear all, close all
mat1=load('data1.mat');
manipulator_endeffector_positions=mat1.manipulator_endeffector_positions;
renovation_cells_manipulatorbase_positions=mat1.renovation_cells_manipulatorbase_positions;
renovation_cells_waypoints_onpath=mat1.renovation_cells_waypoints_onpath;

%% processing procedures
for i=1:1:size(manipulator_endeffector_positions,2)
    for j=1:1:size(manipulator_endeffector_positions{i},2)
        for k=1:1:size(manipulator_endeffector_positions{i}{j},2)
            for m=1:1:size(manipulator_endeffector_positions{i}{j}{k},1)
                if manipulator_endeffector_positions{i}{j}{k}(m,2)>0
                    manipulator_endeffector_positions{i}{j}{k}(m,2)=0.2250;
                end
                if manipulator_endeffector_positions{i}{j}{k}(m,2)<0
                    manipulator_endeffector_positions{i}{j}{k}(m,2)=-0.2250;
                end
            end
        end
    end
end


save("data2.mat",'manipulator_endeffector_positions','renovation_cells_manipulatorbase_positions','renovation_cells_waypoints_onpath')
