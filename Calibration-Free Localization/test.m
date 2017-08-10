%% test for try and catch
try
    a = g;
catch ME
    a = 12;
    rethrow(ME)
end
