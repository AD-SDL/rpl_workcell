from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool

def c2_blank_adjust(**data):
    data_frame = data['data_file']

    def calculate_avg(list):
        """calculate_avg

        Description: Average calculations of the "H" values (blanks/control wells)

        Parameters: 
            list: TODO

        Returns: 
            avg_list: TODO
        
        """
        avg_list = []
        pointer1 = 0
        pointer2 = 6

        while pointer1 < 6:
            
            avg_list.insert(pointer1,(float(list[pointer1]) + float(list[pointer2]))/2)
            avg_list.insert(pointer2,(float(list[pointer1]) + float(list[pointer2]))/2)
            pointer1+=1
            pointer2+=1
        
        return avg_list
    """od_blank_adjusted

        Description: Recives a data_frame and calculates blank adjusted values for each data point

        Parameters: 
            data_frame: Data itself
            time_stamps: A list that contains the time points
        
        Returns: 
            - blank_adj_data_frame: A new data frame with blank adjusted values
            - adjusted_values_list: A list of blank adjusted values (this will be used to insert the values into the database)
    """
    blank_adj_data_frame = data_frame
    adjusted_values_list = []
    for time_point in time_stamps:
    
        blank_adj_list = calculate_avg(list(data_frame[time_point][84:]))
        index = 0
        
        for data_index_num in range(1,len(data_frame)+1) :
            A = float(data_frame[time_point][data_index_num])
            if index == len(blank_adj_list):
                index = 0
            adjust = round(float(data_frame[time_point][data_index_num]) - blank_adj_list[index], 3)
            if adjust < 0:
                adjust = 0
            
            blank_adj_data_frame[time_point][data_index_num] = adjust
            adjusted_values_list.append(adjust)
            index+=1
    
    return blank_adj_data_frame, adjusted_values_list


@generate_flow_definition
class C2_blank_adjust(GladierBaseTool):
    funcx_functions = [c2_blank_adjust]
    required_input = [
        
        'funcx_endpoint_compute'
    ]
