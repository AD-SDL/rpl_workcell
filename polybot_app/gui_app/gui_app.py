import streamlit as st
import pandas as pd
import altair as alt
import numpy as np
from st_aggrid import GridOptionsBuilder, AgGrid, GridUpdateMode, DataReturnMode
import streamlit.components.v1 as components
import colormath
from colormath.color_objects import LabColor, XYZColor, sRGBColor
from colormath.color_conversions import convert_color
from streamlit_plotly_events import plotly_events
import plotly.express as px
from scipy.spatial import distance
from colormap import rgb2hex
import plotly
import json 
import time
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from streamlit_autorefresh import st_autorefresh
import plotly.graph_objects as go
import os
import signal

def update_dataframe(new_data, dataframe):
    return dataframe.append(new_data, ignore_index=True)

def get_hex_color(L, a, b, df=None):
    color_list = []
    y_pred = list(zip(L, a,b))

    for i in range(len(y_pred)):
        lab_list = (list(y_pred[i]))
        a = ', '.join(str(item) for item in list(y_pred[i]))
        lab = LabColor(lab_l = lab_list[0], lab_a = lab_list[1], lab_b = lab_list[2], observer='2', illuminant='d65')
        rgb = convert_color(lab, sRGBColor)
        rgb = colormath.color_objects.sRGBColor(rgb.clamped_rgb_r, rgb.clamped_rgb_g, rgb.clamped_rgb_b, is_upscaled=False)
        try:
            c = rgb.get_rgb_hex()
            color_list.append(str(c))
        except:
            color_list.append(0)
    try:
        df['color_shade'] = color_list
    except:
        pass
    return df, color_list

def get_color_selector(L_value):
    # Number of points
    num_points = 500

    # Generate random radii and angles
    radii = 128 * np.sqrt(np.random.rand(num_points))
    angles = 2 * np.pi * np.random.rand(num_points)
    # Create coordinates using the radii and angles
    a_values = radii * np.cos(angles)
    b_values = radii * np.sin(angles)

    # Create a DataFrame with the L, a and b values
    df = pd.DataFrame({
        'L* (Colored State)':np.repeat(L_value, num_points),
        'a* (Colored State)': a_values,
        'b*(Colored State)': b_values,
    })
    df_random , random_color_list = get_hex_color(df['L* (Colored State)'], df['a* (Colored State)'], df['b*(Colored State)'], df)
    df_random.to_csv('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/color_selection.csv', index=None)
    return df_random

def run_script():
    process = subprocess.Popen(["bash",'/home/rpl/workspace/polybot_workcell/scripts/run_batman_demo.sh'])
    # process =  subprocess.Popen(['nohup', '/home/rpl/workspace/polybot_workcell/scripts/run_batman_demo.sh', '&'])#, shell=True)
    with open('pid.txt', 'w') as f:
        f.write(str(process.pid))

def stop_script():
    with open('pid.txt', 'r') as f:
        pid = f.read().strip()
        if pid:
            os.kill(int(pid), signal.SIGTERM)
            st.write("Process stopped")
        else:
            st.write("No running process found")

def demo_page(init_file=None):
    st.title('ECPs Demo')
    # count = st_autorefresh(interval=2000, limit=100, key="fizzbuzzcounter")

    # Slider to control the L value
    L_value = st.slider('Select a value for the L* coordinate', min_value=0, max_value=100, value=50)

    # # Number of points
    # num_points = 500
    # # Generate random radii and angles
    # radii = 128 * np.sqrt(np.random.rand(num_points))
    # angles = 2 * np.pi * np.random.rand(num_points)
    # # Create coordinates using the radii and angles
    # a_values = radii * np.cos(angles)
    # b_values = radii * np.sin(angles)
    # # print(a_values)
    # # Create a DataFrame with the L, a and b values
    # df = pd.DataFrame({
    #     'L* (Colored State)':np.repeat(L_value, num_points),
    #     'a* (Colored State)': a_values,
    #     'b*(Colored State)': b_values,
    # })

    df_random = pd.read_csv('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/color_selection.csv')
    df_random['L* (Colored State)'] = np.repeat(L_value, df_random.shape[0])
    df_random , random_color_list = get_hex_color(df_random['L* (Colored State)'], df_random['a* (Colored State)'], df_random['b*(Colored State)'], df_random)
    fig = px.scatter(df_random , x='a* (Colored State)', y = 'b*(Colored State)', color='color_shade',
                     color_discrete_map='identity')
    
    fig.add_trace(go.Scatter(x=np.array(36.278424107221376), y=np.array(70.45616081916461) , mode='markers',name='goal',
                marker=dict(color= 'rgba(255, 255, 255, 0)', symbol='star', size=20, line=dict(color='black', width=2))))
    
    import json 
    with open(init_file, 'r') as f:
        init_json_file = json.load(f)

    experiments_df = pd.DataFrame(columns = ["smiles1", "percentage1",
                "smiles2", "percentage2","smiles3", "percentage3", "L*", "a*", "b*"])
    
    if init_json_file["iteration_1"]["measured_lab"]:
        L_values = list(init_json_file["iteration_1"]["measured_lab"]["L"].values())
        a_values = list(init_json_file["iteration_1"]["measured_lab"]["a"].values())
        b_values = list(init_json_file["iteration_1"]["measured_lab"]["b"].values())
        smiles1 = list(init_json_file["iteration_1"]["ml_suggestions"]["smiles1"].values())
        percentage1 = list(init_json_file["iteration_1"]["ml_suggestions"]["percentage_1"].values())
        smiles2= list(init_json_file["iteration_1"]["ml_suggestions"]["smiles2"].values())
        percentage2 = list(init_json_file["iteration_1"]["ml_suggestions"]["percentage_2"].values())
        smiles3 = list(init_json_file["iteration_1"]["ml_suggestions"]["smiles3"].values())
        percentage3 = list(init_json_file["iteration_1"]["ml_suggestions"]["percentage_3"].values())

        _ , color_list = get_hex_color(L_values , a_values, b_values)
        # print('color_list', a_values, b_values, color_list )
        fig.add_trace(go.Scatter(x=a_values, y=b_values , mode='markers',name='loop 1',
                marker=dict(color= color_list, symbol='triangle-up', size=15, line=dict(color='black', width=2))))
        data = {"smiles1": smiles1, "percentage1":percentage1,
                "smiles2": smiles2, "percentage2":percentage2,
                "smiles3": smiles3, "percentage3":percentage3,
                "L*":L_values, "a*":a_values, "b*":b_values

        }
        experiments_df = pd.DataFrame(data)
              

    if init_json_file["iteration_2"]["measured_lab"]:
        L_values = list(init_json_file["iteration_2"]["measured_lab"]["L"].values())
        a_values = list(init_json_file["iteration_2"]["measured_lab"]["a"].values())
        b_values = list(init_json_file["iteration_2"]["measured_lab"]["b"].values())
        smiles1 = list(init_json_file["iteration_2"]["ml_suggestions"]["smiles1"].values())
        percentage1 = list(init_json_file["iteration_2"]["ml_suggestions"]["percentage_1"].values())
        smiles2= list(init_json_file["iteration_2"]["ml_suggestions"]["smiles2"].values())
        percentage2 = list(init_json_file["iteration_2"]["ml_suggestions"]["percentage_2"].values())
        smiles3 = list(init_json_file["iteration_2"]["ml_suggestions"]["smiles3"].values())
        percentage3 = list(init_json_file["iteration_2"]["ml_suggestions"]["percentage_3"].values())

        _ , color_list = get_hex_color(L_values , a_values, b_values)
        # print('color_list', a_values, b_values, color_list )
        fig.add_trace(go.Scatter(x=a_values, y=b_values , mode='markers', name='loop 2',
                marker=dict(color= color_list, symbol='circle', size=12, line=dict(color='black', width=2))))
        data = pd.DataFrame({"smiles1": smiles1, "percentage1":percentage1,
                "smiles2": smiles2, "percentage2":percentage2,
                "smiles3": smiles3, "percentage3":percentage3,
            "L*":L_values, "a*":a_values, "b*":b_values })
        experiments_df = update_dataframe(data, experiments_df)

        
    if init_json_file["iteration_3"]["measured_lab"]:
        L_values = list(init_json_file["iteration_3"]["measured_lab"]["L"].values())
        a_values = list(init_json_file["iteration_3"]["measured_lab"]["a"].values())
        b_values = list(init_json_file["iteration_3"]["measured_lab"]["b"].values())
        smiles1 = list(init_json_file["iteration_3"]["ml_suggestions"]["smiles1"].values())
        percentage1 = list(init_json_file["iteration_3"]["ml_suggestions"]["percentage_1"].values())
        smiles2= list(init_json_file["iteration_3"]["ml_suggestions"]["smiles2"].values())
        percentage2 = list(init_json_file["iteration_3"]["ml_suggestions"]["percentage_2"].values())
        smiles3 = list(init_json_file["iteration_3"]["ml_suggestions"]["smiles3"].values())
        percentage3 = list(init_json_file["iteration_3"]["ml_suggestions"]["percentage_3"].values())
        _ , color_list = get_hex_color(L_values , a_values, b_values)
        fig.add_trace(go.Scatter(x=a_values, y=b_values , mode='markers', name='loop 3',
                marker=dict(color= color_list, symbol='square', size=12, line=dict(color='black', width=2))))
        data = pd.DataFrame({"smiles1": smiles1, "percentage1":percentage1,
                "smiles2": smiles2, "percentage2":percentage2,
                "smiles3": smiles3, "percentage3":percentage3,
            "L*":L_values, "a*":a_values, "b*":b_values

        })
        # experiments_df = experiments_df.append(data, ignore_index=True)
        experiments_df = update_dataframe(data, experiments_df)

    if init_json_file["iteration_4"]["measured_lab"]:
        L_values = list(init_json_file["iteration_4"]["measured_lab"]["L"].values())
        a_values = list(init_json_file["iteration_4"]["measured_lab"]["a"].values())
        b_values = list(init_json_file["iteration_4"]["measured_lab"]["b"].values())
        smiles1 = list(init_json_file["iteration_4"]["ml_suggestions"]["smiles1"].values())
        percentage1 = list(init_json_file["iteration_4"]["ml_suggestions"]["percentage_1"].values())
        smiles2= list(init_json_file["iteration_4"]["ml_suggestions"]["smiles2"].values())
        percentage2 = list(init_json_file["iteration_4"]["ml_suggestions"]["percentage_2"].values())
        smiles3 = list(init_json_file["iteration_4"]["ml_suggestions"]["smiles3"].values())
        percentage3 = list(init_json_file["iteration_4"]["ml_suggestions"]["percentage_3"].values())
        _ , color_list = get_hex_color(L_values , a_values, b_values)
        fig.add_trace(go.Scatter(x=a_values, y=b_values , mode='markers',
                marker=dict(color= color_list, symbol='triangle-up', size=12, line=dict(color='black', width=2))))
        data = {"smiles1": smiles1, "percentage1":percentage1,
                "smiles2": smiles2, "percentage2":percentage2,
                "smiles3": smiles3, "percentage3":percentage3,
            "L*":L_values, "a*":a_values, "b*":b_values

        }
        # experiments_df = experiments_df.append(data, ignore_index=True)
        experiments_df = update_dataframe(data, experiments_df)

    # fig.layout.update(showlegend=False)
    # fig.update_traces(marker={'size': 15})

    clicked_point = plotly_events(fig, click_event=True, hover_event=False)
    
    
    target_lab =[L_value ]
    if clicked_point:
        target_lab.append(clicked_point[0]['x'])
        target_lab.append(clicked_point[0]['y'])
        # st.write("The selected value is: ", target_lab)

    st.markdown("""
        * Use the menu at left to select the experimental parameters
        * Select the color shade you want to achieve
        * A table with the six suggested experiments will be shown below
        """)

    st.sidebar.markdown("## Select Inventory")
    strings = ['ProDOT-2Br', 'Ben-2Br', 'BTD-2Br', 'DMoT-2Br', 'EDOT-2Br']
    monomers_dict = {"ProDOT-2Br": "Cc1sc(C)c2OCC(COCC(CC)CCCC)(COCC(CC)CCCC)COc12", 
                     "Ben-2Br":"C1=CC(=CC=C1C)C", "BTD-2Br":"Cc1ccc(C)c2nsnc12", 
                     "DMoT-2Br": "Cc1sc(C)c(OC)c1OC", "EDOT-2Br":"Cc1sc(C)c2OCCOc12"}
    # Select monomers
    monomers_list=[]
    for s in strings:
        checkbox = st.sidebar.checkbox(s)
        if checkbox:
            monomers_list.append(monomers_dict[s])

    # Select number of monomers
    num_monomers_list=[]
    st.sidebar.markdown("## Number of monomers")
    strings = ['two', 'three']
    num_dict={'two':2, 'three':3}
    for s in strings:
        checkbox = st.sidebar.checkbox(s)
        if checkbox:
            num_monomers_list.append(num_dict[s])

    # Select ratio step
    st.sidebar.markdown("## Monomer ratio step")
    step = st.sidebar.slider('Select the ratio steps', 1, 10)
    
    # initialize the instructions json
    init_json ={
        "monomers": monomers_list,
        "target_Lab": target_lab,
        "number_of_monomers": num_monomers_list,
        "ratio_step": step,

        "iteration_1":{
            "ml_suggestions": [],
            "predicted_lab": [],
            "uv_vis_data": [],
            "measured_lab": []},

        "iteration_2":{
            "ml_suggestions": [],
            "predicted_lab": [],
            "uv_vis_data": [],
            "measured_lab": []},

        "iteration_3":{
            "ml_suggestions": [],
            "predicted_lab": [],
            "uv_vis_data": [],
            "measured_lab": []},

        "iteration_4":{
            "ml_suggestions": [],
            "predicted_lab": [],
            "uv_vis_data": [],
            "measured_lab": []},
    }
    
    if not experiments_df.empty:        
        st.write(experiments_df)
        experiments_df.to_csv('all_results.csv')

    if st.button('Send instructions', key =0):
        import json
        with open('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json', 'w', encoding='utf-8') as f:
            json.dump(init_json, f, ensure_ascii=False, indent=4)
        # save the inventory and the user selected Lab to the json C:\Users\kvriz\Desktop\ECP_demo_ML_code\demo_files\init.json

    if st.button('Run experiments', key =1):
        import json
        with open('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json', 'w', encoding='utf-8') as f:
            json.dump(init_json, f, ensure_ascii=False, indent=4)
        # save the inventory and the user selected Lab to the json C:\Users\kvriz\Desktop\ECP_demo_ML_code\demo_files\init.json
        print('Experiment is running')
        # This generates the json file with the color coordinates and calls the demo_app.py 
        # subprocess.Popen(['nohup', '/home/rpl/workspace/polybot_workcell/scripts/run_batman_demo.sh', '&'])#, shell=True)
        run_script()
        try:
            st.write(experiments_df)
        except:
            pass
        
    if st.button('Stop script', key =2):
        stop_script()
    #if __name__ == "__main__":

init_file = '/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json'
demo_page(init_file)


#TODO show the selected Lab value from the user