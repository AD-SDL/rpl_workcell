# E.g., python draw.py -i color_picker.json -o i.pdf

from PIL import Image, ImageDraw, ImageFont, ImageColor
import json, sys, argparse
  
white = (255, 255, 255)
black = (0, 0, 0)
blue  = ImageColor.getrgb('lightblue')
green = ImageColor.getrgb('lightgreen')
red   = ImageColor.getrgb('pink')
orange= ImageColor.getrgb('orange')

base_color = white
text_x_offset=8
text_offset=5
text_y_offset=3
text_height=15
arrow_size=10
box_height=text_height+2*text_y_offset
box_sep   = 10
first_item_offset = 25
fontsize=14
x_size = 1000
y_size = 1200
action_name_length = 100

python_offset = 0
workflow_offset = 200
action_offset = 400
instrument_offset = action_offset + action_name_length + text_x_offset
protocol_offset = instrument_offset+150

x_max = 0
y_max = 0

# Create canvas
im = Image.new('RGB', (x_size, y_size), base_color)
draw = ImageDraw.Draw(im)

arial = ImageFont.truetype('arial.ttf', fontsize)
arial_bold = ImageFont.truetype('arial-bold.ttf', fontsize)

def draw_bold_text(x, y, text):
    draw.text((x, y), text, font=arial_bold, fill=black)

def draw_my_text(x, y, A, B):
    width = arial.getlength(A)+text_offset
    draw.text((x-width, y), A, font=arial, fill=black)
    draw.text((x, y), B, font=arial_bold, fill=black)

def draw_my_text_normal(x, y, A):
    draw.text((x, y), A, font=arial, fill=black)

def draw_my_text_rectangle(x, y, text, color):
    # x0, y0, x1, y1
    text_width = arial.getlength(text)
    width = text_width+2*text_x_offset
    draw.rectangle((x, y, x+width, y+box_height), fill=color, outline=white)
    draw.text((x+text_x_offset, y+text_y_offset), text, font=arial, fill=black)
    global y_max
    if y+box_height > y_max:
        y_max = y+box_height
    global x_max
    if x+width > x_max:
        x_max = x+width
    return((x, y, x+width, y+box_height))
    
def draw_my_text_rectangle_width(x, y, length, text, color):
    # x0, y0, x1, y1
    text_width = arial.getlength(text)
    width = text_width+2*text_x_offset
    text_start = length/2 - text_width/2
    draw.rectangle((x, y, x+length, y+box_height), fill=color, outline=white)
    draw.text((x+text_start, y+text_y_offset), text, font=arial, fill=black)
    global y_max
    if y+box_height > y_max:
        y_max = y+box_height
    global x_max
    if x+length > x_max:
        x_max = x+length
    return((x, y, x+length, y+box_height))

def draw_header():
    draw_my_text(python_offset, 0, '', 'Python program')
    draw_my_text(workflow_offset, 0, 'runs', 'Workflow (YAML)')
    draw_my_text(action_offset, 0, 'invokes', 'Action')
    draw_my_text(instrument_offset, 0, 'on', 'Instrument')
    draw_my_text(protocol_offset, 0, 'with', 'OT2 protocol specification')
    global x_max
    x_max = protocol_offset + arial_bold.getlength('OT protocol specification') + text_x_offset

def arrow_from_to(from_box, to_box, color):
    (x_from_1, _, x_from_2, y_from) = from_box
    (x_to_1, y_to_1, _, y_to_2) = to_box
    x_start = x_from_1+(x_from_2-x_from_1)/2
    y_end   = y_to_1+(y_to_2-y_to_1)/2
    draw.line( (x_start, y_from, x_start, y_end), fill=color, width=2)
    draw.line( (x_start, y_end, x_to_1, y_end), fill=color, width=2)
    draw.polygon([(x_to_1,y_end), (x_to_1-arrow_size, y_end-arrow_size/2), (x_to_1-arrow_size, y_end+arrow_size/2)], fill=color)

def draw_actions(actions):
    for (t, y) in actions:
        draw_my_text_rectangle(action_offset, y, t, red)

def draw_workflow(workflow, y_start):
    y_size = 0
    name = workflow['name']
    actions = workflow['actions']
    name_box = draw_my_text_rectangle(workflow_offset, y_start+y_size, name, green)
    y_size += box_height
    for action in actions:
        action_name = action['name']
        instrument  = action['instrument']
        text_box = draw_my_text_rectangle_width(action_offset, y_start+y_size, action_name_length, action_name, red)
        draw_my_text_normal(action_offset+action_name_length+text_x_offset,  y_start+y_size+text_y_offset, instrument)
        arrow_from_to(name_box, text_box, red)
        try:
            protocol = action['protocol']
            y_size += box_height
            protocol_box = draw_my_text_rectangle(protocol_offset, y_start+y_size, protocol, orange)
            arrow_from_to(text_box, protocol_box, orange)
        except:
            pass
        y_size += box_height + box_sep
    return (name_box, y_size)

def process_file(spec_file, output_file):

    draw_header()

    f = open(spec_file, 'r')
    plot_spec = json.load(f)
    plot = plot_spec['python']
    python_program = plot['name']
    loop_spec = plot['workflows']

    python_y_location = 30
    python_box = draw_my_text_rectangle(python_offset, python_y_location, python_program, blue)

    y_location = python_y_location+first_item_offset
    for workflow in loop_spec:
        (workflow_box, y_size) = draw_workflow(workflow, y_location)
        arrow_from_to(python_box, workflow_box, green)
        y_location += y_size

    im_cropped = im.crop((0, 0, x_max, y_max))

    im_cropped.save(output_file, quality=95)


def main(argv):
    parser = argparse.ArgumentParser(description='Program to generate plots for WEI workflows')
    parser.add_argument('-i','--input', help='Input file name', required=True)
    parser.add_argument('-o', '--output', default="t.pdf", help='Output file name', required=True)
 
    args = parser.parse_args()

    process_file(args.input, args.output)

if __name__ == "__main__":
   main(sys.argv[1:])
