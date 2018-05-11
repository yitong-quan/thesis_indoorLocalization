main functions: 
	'KF_using_HTerm_data.m' -- to recover the trajectory of the tag, based on known information of nodes' postions
	'cali_free_lsqnonlin_DistData_2018_Hangar_fullData_workingOn.m'  -- to recover the the trajectory of the tag and the positions of the node simutanously




Order of running files to output trajectroy from distance data from HTerm recorded .log files.

copy the original 'XXX.log' file, so another file 'XXX - Copy.log' is generated.
>>
refine (input) the 'XXX - Copy.log' file by 'data_format_refine_HTerm.m', and get (output) 'XXX - Copy_refined.log'
>>
generate matrix formate data of time and distance by 'read_file_gene_matr__with_timeStmp.m' using XXX - Copy_refined.log as input, and save it with self defined name, i.e. 'xxx.mat'.
>>
import 'xxx.mat' and run using 'KF_using_HTerm_data.m'


Note that the postions of the nodes should be fed into to the 'KF_using_HTerm_data.m',
if the nodes are out of the rang of MoCap system, record the distances between each pair of nodes and then use the optimization method such as shown in 'determineNodesPositionBaseOnDistToEachOthers.m' to recover the position.
if the nodes are inside the rang of MoCap system, feel free to use the position infomation from the MoCap system.