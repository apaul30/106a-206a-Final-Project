from ursina import *
import json

app = Ursina()
window.color = color.rgb(235, 238, 247)  

n = 10
matrix = [[[None for _ in range(n)] for _ in range(n)] for _ in range(n)]


cube_list = []
floor = Entity(model='plane', scale=n, texture='white_cube', texture_scale=(n, n), collider='box')
floor.y = -.5
floor.x = .5
floor.z = .5

# make the (0,0,0) floor tile red
red_tile = Entity(model='quad', scale=(1, 1), color=color.red)
red_tile.rotation_x = 90  # Rotate 90 degrees along the X-axis
red_tile.x = 0
red_tile.y = -.45
red_tile.z = 0

red_tile = Entity(model='quad', scale=(1, 1), color=color.blue)
red_tile.rotation_x = 90  # Rotate 90 degrees along the X-axis
red_tile.x = 1
red_tile.y = -.45
red_tile.z = 0

red_tile = Entity(model='quad', scale=(1, 1), color=color.green)
red_tile.rotation_x = 90  # Rotate 90 degrees along the X-axis
red_tile.x = 0
red_tile.y = -.45
red_tile.z = 1


def input(key):
    if key == 'left mouse down':
        position = mouse.world_point
        position = round(mouse.world_point[0]), round(mouse.world_point[1]), round(mouse.world_point[2])
        i = round(position[1])
        position_pos = [round(position[0] * -1), round(position[1]), round(position[2] * -1)]
        while matrix[position_pos[0]][i][position_pos[2]] != None:
            i += 1
            position_pos = [round(position[0] * -1), round(position[1]), round(position[2] * -1)]

        cube = Entity(
                model='cube',
                color=color.orange,
                texture='white_cube',
                scale=1,
                position=(-position_pos[0], i, -position_pos[2]),
                collider='box'
            )
        matrix[position_pos[0]][i][position_pos[2]] = cube
        position = position[0], i, position[2]
        

editor_camera = EditorCamera()
def update():
    speed = 1.0  
    if held_keys['a']:
        editor_camera.rotation_y -= speed
    if held_keys['d']:
        editor_camera.rotation_y += speed
    if held_keys['w']:
        editor_camera.rotation_x -= speed
    if held_keys['s']:
        editor_camera.rotation_x += speed
    if held_keys['q']:
        exit(0)

    if held_keys['e']:
        export()
        print('exported')
        time.sleep(1)
        exit(0)

def export():
    layer_list = []
    for i in range(n):
        for j in range(n):
            for k in range(n):
                if matrix[i][j][k] != None:
                    layer_list.append([i, j, k])
    # save to file
    data = {'data': layer_list}
    with open('model.json', 'w') as f:
        json.dump(data, f)
        
    return layer_list

app.run()