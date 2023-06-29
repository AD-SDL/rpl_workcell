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
inter_column_width=40
box_height=text_height+2*text_y_offset
box_sep   = 10
first_item_offset = 35
fontsize=14
x_size = 1000
y_size = 1200
action_name_length = 100
python_offset = 0
max_step_width = 80

x_max = 0
y_max = 0

# Create canvas
im = Image.new('RGB', (x_size, y_size), base_color)
draw = ImageDraw.Draw(im)

font = ImageFont.truetype('arial.ttf', fontsize)
arial = ImageFont.truetype('arial.ttf', fontsize)
arial_bold = ImageFont.truetype('arial-bold.ttf', fontsize)


def draw_bold_text(x, y, text):
    draw.text((x, y), text, font=arial_bold, fill=black)


def draw_text(x, y, A):
    draw.text((x, y), A, font=arial, fill=black)
    global x_max
    if x+arial.getsize(A)[0] > x_max:
        x_max = x + arial.getsize(A)[0]

def draw_my_text_rectangle(x, y, text, color):
    width = arial.getsize(text)[0] + 2*text_x_offset
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
    text_start = length/2 - arial.getsize(text)[0]/2
    draw.rectangle((x, y, x+length, y+box_height), fill=color, outline=white)
    draw.text((x+text_start, y+text_y_offset), text, font=arial, fill=black)
    global y_max
    if y + box_height > y_max:
        y_max = y + box_height
    global x_max
    if x + length > x_max:
        x_max = x + length
    return((x, y, x+length, y+box_height))


def draw_header(offsets):
    draw_bold_text(offsets['python_offset'], 0, 'Python')
    draw_bold_text(offsets['workflow_offset'], 0, 'Workflow')
    draw_bold_text(offsets['action_offset'], 0, 'Action')
    draw_bold_text(offsets['component_offset'], 0, 'Module [: Transfer]')

def arrow_from_to(destination, from_box, to_box, color, factor):
    (x_from_1, _, x_from_2, y_from) = from_box
    (x_to_1, y_to_1, x_to_2,  y_to_2) = to_box
    x_start = x_from_1+(x_from_2-x_from_1)/2
    x_start2 = x_to_1+(x_to_2-x_to_1)/2
    y_end   = y_to_1+(y_to_2-y_to_1)/2
    if destination=='side':
        draw.line( (x_start, y_from, x_start, y_end), fill=color, width=2)
        draw.line( (x_start, y_end, x_to_1, y_end), fill=color, width=2)
        draw.polygon([(x_to_1,y_end), (x_to_1-arrow_size, y_end-arrow_size/2), (x_to_1-arrow_size, y_end+arrow_size/2)], fill=color)
    elif destination=='horizontal':
        draw.line( (x_from_2, y_end, x_to_1, y_end), fill=color, width=2 )
        draw.polygon([(x_to_1,y_end), (x_to_1-arrow_size, y_end-arrow_size/2), (x_to_1-arrow_size, y_end+arrow_size/2)], fill=color)
    elif destination=='vertical':
        draw.line( (x_start2, y_from, x_start2, y_to_1), fill=color, width=2 )
        draw.polygon([(x_start2-arrow_size/2/factor,y_to_1-arrow_size/factor), (x_start2+arrow_size/2/factor,y_to_1-arrow_size/factor), (x_start2, y_to_1)], fill=color)
    else:
        draw.line( (x_start, y_from, x_start, y_to_1), fill=color, width=2)


def draw_workflow(offsets, workflow, y_start):
    y_size = 0
    max_size = offsets['max_wf_width'] + 2*text_x_offset
    name = workflow['name'].replace('.yaml', '')

    try:
        extras = workflow['repeat']
        return_box = draw_my_text_rectangle_width(offsets['python_step_offset'], y_start+y_size, offsets['max_step_width'], 'Repeat:', blue)
        repeat_box = return_box
        repeat_indent = 20
        y_size += box_height + 15
        run_box = draw_my_text_rectangle_width(offsets['python_step_offset'] + repeat_indent, y_start+y_size, offsets['max_step_width'], 'Run:', blue)
        (_, _, _, y_repeat) = repeat_box
        arrow_from_to('vertical', repeat_box, run_box, blue, 2)
        extra_y = y_size + box_height + 15
        previous_extra_box = run_box
        for extra in extras:
            extra_box = draw_my_text_rectangle_width(offsets['python_step_offset'] + repeat_indent, y_start+extra_y, offsets['max_step_width'], extra, blue)
            extra_y += box_height + 15
            if previous_extra_box != '':
                arrow_from_to('vertical', previous_extra_box, extra_box, blue, 2)
            previous_extra_box = extra_box
        last_box = previous_extra_box

        # Draw return arrow
        factor = 2
        (x1, y1, x2, y2) = previous_extra_box
        (_, _, _, y_repeat) = repeat_box
        x_left = x1-8
        draw.line( ((x1+x2)/2, y2, (x1+x2)/2, y2+8), fill=blue, width=2)
        draw.line( ((x1+x2)/2, y2+8, x_left, y2+8), fill=blue, width=2)
        draw.line( (x_left, y2+8, x_left, y_repeat), fill=blue, width=2)
        draw.polygon([(x_left, y_repeat), (x_left-arrow_size/2/factor, y_repeat+arrow_size/factor), (x_left+arrow_size/2/factor, y_repeat+arrow_size/factor)], fill=blue)
    except:
        repeat_indent = 0
        return_box = draw_my_text_rectangle_width(offsets['python_step_offset'] + repeat_indent, y_start+y_size, offsets['max_step_width'], 'Run:', blue)
        run_box = return_box
        last_box = return_box

    name_box = draw_my_text_rectangle_width(offsets['workflow_offset'], y_start+y_size, max_size, name, green)
    arrow_from_to('horizontal', run_box, name_box, green, 1)
    y_size += box_height

    actions = workflow['actions']
  
    for action in actions:
        action_name = action['name']
        instrument  = action['instrument']
        text_box = draw_my_text_rectangle_width(offsets['action_offset'], y_start+y_size, offsets['max_action_width'], action_name, red)
        draw_text(offsets['component_offset'], y_start + y_size + text_y_offset, instrument)

        try:
            from_locn = action['from']
            to_locn   = action['to']
            text = f': {from_locn}\u2192{to_locn}'
            draw_text(offsets['component_offset'] + arial.getsize(instrument)[0],  y_start+y_size+text_y_offset, text)
        except:
            pass

        arrow_from_to('side', name_box, text_box, red, 1)

        try:
            protocol = action['protocol'].replace('.yaml', '')
            y_size += box_height+10
            protocol_size = arial.getsize(protocol)[0]
            protocol_box = draw_my_text_rectangle_width(offsets['action_offset']+20, y_start+y_size, protocol_size+2*text_x_offset, protocol, orange)
            arrow_from_to('top', text_box, protocol_box, orange, 1)
        except:
            pass
        y_size += box_height + box_sep
    return (return_box, y_size)

def get_sizes(plot):
    n = plot['name'].replace('.py','')
    python_width = max(arial.getsize(n)[0], 20+max_step_width)
    max_workflow_name_width = max([arial.getsize(x['name'].replace('.yaml',''))[0] for x in plot['workflows']])
    workflows = plot['workflows']
    actions = sum([x['actions'] for x in workflows], [])
    max_action_name_width = max([arial.getsize(x['name'])[0] for x in actions])

    workflow_header_width = arial_bold.getsize('Workflow')[0]
    action_offset = python_offset + python_width + inter_column_width + max(max_workflow_name_width, workflow_header_width) + inter_column_width
    max_action_width = max_action_name_width+2*text_x_offset
    offsets = { 'python_offset'   : 0,
                # 20 is repeat_indent
                'workflow_offset' : python_offset + python_width + 1.5*inter_column_width,
                'python_step_offset' : python_offset + 40,
                'python_repeat_offset' : python_offset + 60 + 40,
                'max_step_width'  : max_step_width,
                'max_wf_width'    : max_workflow_name_width,
                'action_offset'   : action_offset,
                'max_action_width': max_action_width,
                'component_offset': action_offset + max_action_width + text_x_offset/2
              }

    return( offsets )


def process_file(spec_file, output_file):
    f = open(spec_file, 'r')
    plot_spec = json.load(f)

    offsets = get_sizes(plot_spec['python'])

    draw_header(offsets)

    plot = plot_spec['python']
    python_program = plot['name'].replace('.py','')
    loop_spec = plot['workflows']

    python_y_location = 20
    python_box = draw_my_text_rectangle(python_offset, python_y_location, python_program, blue)

    y_location = python_y_location + first_item_offset

    for workflow in loop_spec:
        (workflow_box, y_size) = draw_workflow(offsets, workflow, y_location)
        (x1, y1, x2, y2) = python_box
        arrow_from_to('side', (x1, y1, x1+40, y2) , workflow_box, blue, 1)
        y_location += y_size

    im_cropped = im.crop((0, 0, x_max, y_max))
    im_cropped.save(output_file)

def main(argv):
    parser = argparse.ArgumentParser(description='Program to generate plots for WEI workflows')
    parser.add_argument('-i','--input', help='Input file name', required=True)
    parser.add_argument('-o', '--output', default="t.pdf", help='Output file name', required=True)

    args = parser.parse_args()

    process_file(args.input, args.output)

if __name__ == "__main__":
   main(sys.argv[1:])

