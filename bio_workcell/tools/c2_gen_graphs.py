from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool

def c2_gen_graphs(**data):
    import matplotlib.pyplot as plt
    import os
    import pandas as pd
    """generate_graphs

    Description: Received a blank adjusted data frame and produces one graph per data timepoint

    Parameters: 
        blank_adj_df: blank adjusted data in data frame format
        data_filename: file name of the original data csv file without extension (not file path)
        plot_directory_path: path to the directory where all created graphs should be stored 

    Returns: 
        plot_paths: a list of file paths to newly saved graphs in order of increasing data timepoint
    
    """
    csv_file = data.get("proc_folder") +"/" + data.get('csv_file')
    plot_directory_path = data.get("proc_folder")
    ba_csv_file =  data.get("proc_folder") + "/blank_adj_" + data.get('csv_file')
    data_filename = data.get('csv_file').split('.')[0]
#    blank_adj_df, data_filename, plot_directory_path
    plot_paths = []

    # define x-axis and x-axis ticks
    x_axis = []
    x_rep1 = [1,2,3,4,5,6]
    x_rep2 = [7,8,9,10,11,12]

    x_ticks = x_rep1.copy()
    for x in x_rep2:
        x_ticks.append(x)

    for i in range(8): 
        x_axis.append(x_rep1)
        x_axis.append(x_rep2)
    
    # define plot colors and figure dimensions
    colors = ["b","g","r","c","m","y",'tab:brown', "k"]
    blank_adj_df = pd.read_csv(ba_csv_file)
    # Graph data from each timepoint
    for timepoint in blank_adj_df.columns[3:]:
        data_list = blank_adj_df[timepoint].tolist()
        data_list = [float(x) for x in data_list]

        # determine plot title and file path
        plot_title = f"{data_filename}, timepoint = {timepoint} (seconds)"
        plot_basename = f"{data_filename}_{str(timepoint).split('.')[0]}.png"
        
        try:
            plot_file_path = os.path.join(plot_directory_path, plot_basename)
            plot_paths.append(plot_file_path)
        except OSError as e: 
            print(e)

        data_in_sets = []
        data_rep1 = []
        data_rep2 = []

        # separate data into replicate 1 and 2
        while not len(data_list) == 0:  
            data_in_sets.append(data_list[:6])
            data_in_sets.append(data_list[6:12])
            data_rep1.append(data_list[:6])
            data_rep2.append(data_list[6:12])
            data_list = data_list[12:]

        plt.figure(figsize=(10, 5))

        # plot each replicate separately (to maintain color across replicates)
        for i in range(len(data_rep1)): 
            plt.plot(x_rep1, data_rep1[i], colors[i])
        for i in range(len(data_rep2)): 
            plt.plot(x_rep2, data_rep2[i], colors[i])

        plt.legend(["Row A","Row B","Row C","Row D","Row E","Row F","Row G","Row H"], loc='center left', bbox_to_anchor=(1, 0.5), borderaxespad=0.)  #TODO: extract strain names from db
        plt.title(plot_title)
        plt.xticks(x_ticks)
        plt.xlabel("Dilution (1-6) and (7-12) are replicates. 1 is most concentrated, 6 is no treatment")
        plt.ylabel("Blank-adjusted OD(590)")
    
        # save figure to file
        plt.savefig(plot_file_path)
        plt.close() 

    return plot_paths


@generate_flow_definition
class C2_gen_graphs(GladierBaseTool):
    funcx_functions = [c2_gen_graphs]
    required_input = [
        
        'funcx_endpoint_compute'
    ]
