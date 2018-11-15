This project is about indoor localization using Ultra-wideband radio signal.

The most important codes are: 
    - C for micro-controllers on a ranging system to measure distances on schedule,
    - MATLAB for algorithms to locate the moving target based on its distances to anchor nodes. The algorithms include: 
        -- EKF variances, with the positions of the anchor nodes as prior knowledge, 
        -- an algorithm based on the optimization method, without the prior knowledge about the positions of the anchor nodes,
        -- an algorithm with non-line-of-sight error mitigation functionality.

I suggest the readers go directly to a subfolder, as this folder contains the code together with the finest comments. This folder is in the followed link: 'https://github.com/yitong-quan/thesis_indoorLocalization/tree/master/data-from-experiments/experiment_09.02.2018.Hangar/09%2CFeb%2C2018Hangar/algo'.