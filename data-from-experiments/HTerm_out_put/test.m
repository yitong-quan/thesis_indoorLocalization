%% test for the node and tag position

function test(Dist)
    positionOfNodes = [2938.41844377029,-3013.26989169788; [184.822603869210,-143.127276884650];...
        [4161.77182689655,2235.61214448276]; [-1396.30540772784,-2433.57009459426]; [-1741.84732630000,2032.12649985000]]'/1000;
    plot(positionOfNodes(1,:),positionOfNodes(2,:),'*'); 
    hold on;
    pause(2)
    for j = 1:length(Dist)
        for i = 1 : size(positionOfNodes, 2)
            h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), Dist(j,i));
        end   
        %axis square;  %axis tight;
        axis([-6 8 -6 6])
        pause(0.5)
        delete(h)
    end

end

function h_circle = plotCircle(x,y,r)
    if ~isnan(r)
        %ang=ang_start:0.01:ang_end;
        ang=0:0.01:2*pi;
        xp=r*cos(ang);
        yp=r*sin(ang);
        h_circle = plot(x+xp,y+yp);
    else
        h_circle = plot(0,0,'bo');
    end
end