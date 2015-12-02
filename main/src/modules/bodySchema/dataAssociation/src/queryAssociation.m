%% Make query for data association
close all


query_types = {'joint','touch','word'};

query = input('Enter one modality [joint,touch,word]: ', 's');

switch query
    case 'joint'
        joint_idx = input('Enter joint index [1,...,7]: ');
        dataAssoc = dataAssocMtx(joint_idx,:,:);
        dataAssoc = reshape(dataAssoc,3,3);
        figure;hold on;grid on
        for y = 1:3
            for z = 1:3
                for x = 1:7
                    plot3(x,y,z,'ro','MarkerSize', dataAssocMtx(x,y,z)*100+1, 'LineWidth',2);
                end
                plot3(joint_idx,y,z,'g*','MarkerSize', 10, 'LineWidth',2);
            end
        end
        xlabel('joint index');ylabel('words');zlabel('touch index');colormap(winter)
        
        [I,J] = find(dataAssoc);
        if (~isempty(I) && ~isempty(J))
            word_assoc = I';
            touch_assoc = J';
            disp(['Word involved: ', num2str(unique(word_assoc))])
            disp(['Touch involved: ', num2str(unique(touch_assoc))])
        end
        
        
    case 'touch'
        touch_idx = input('Enter touch index [1,2,3]: ');
        dataAssoc = dataAssocMtx(:,:,touch_idx);
        dataAssoc = reshape(dataAssoc,7,3);
        figure;hold on;grid on
        for y = 1:3            
            for x = 1:7
                for z = 1:3
                    plot3(x,y,z,'ro','MarkerSize', dataAssocMtx(x,y,z)*100+1, 'LineWidth',2);
                end
                plot3(x,y,touch_idx,'g*','MarkerSize', 10, 'LineWidth',2);
            end
        end
        xlabel('joint index');ylabel('words');zlabel('touch index');colormap(winter)
        
        [I,J] = find(dataAssoc);
        if (~isempty(I) && ~isempty(J))
            joints_assoc = I';
            word_assoc = J';
            disp(['Joints involved: ', num2str(joints_assoc)])
            disp(['Word involved: ', num2str(unique(word_assoc))])
        end
        
        
    case 'word'
        word_idx = input('Enter word index [1,2,3]: ');
        dataAssoc = dataAssocMtx(:,word_idx,:);
        dataAssoc = reshape(dataAssoc,7,3);
        figure;hold on;grid on
        for x = 1:7            
            for z = 1:3
                for y = 1:3
                    plot3(x,y,z,'ro','MarkerSize', dataAssocMtx(x,y,z)*100+1, 'LineWidth',2);
                end
                plot3(x,word_idx,z,'g*','MarkerSize', 10, 'LineWidth',2);
            end
        end
        xlabel('joint index');ylabel('words');zlabel('touch index');colormap(winter)
        
        [I,J] = find(dataAssoc);
        if (~isempty(I) && ~isempty(J))
            joints_assoc = I';
            touch_assoc = J';
            disp(['Joints involved: ', num2str(joints_assoc)])
            disp(['Touch involved: ', num2str(unique(touch_assoc))])
        end
        
end
