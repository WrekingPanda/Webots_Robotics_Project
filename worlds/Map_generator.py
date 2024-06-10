import numpy as np
import csv
from PIL import Image


def write_square_arena():
    f.write('RectangleArena {')
    f.write('  translation ' + str(resolution*width/2) + ' ' + str(resolution*height/2) + ' 0.0')
    f.write('  floorSize ' + str(resolution*width) + ' ' + str(resolution*height))
    f.write('  floorTileSize 0.25 0.25')
    f.write('  floorAppearance Parquetry {')
    f.write('    type "light strip"')
    f.write('  }')
    f.write('  wallHeight 0.05')
    f.write('}')

def write_circular_arena():
    f.write('CircularArena {')
    f.write('  translation ' + str(resolution*width/2) + ' ' + str(resolution*height/2) + ' 0.0')
    f.write('  radius 1')
    f.write('  floorTileSize 0.25 0.25')
    f.write('  floorAppearance Parquetry {')
    f.write('    type "light strip"')
    f.write('  }')
    f.write('  wallHeight 0.05')
    f.write('}')


def choose_arena():
    valid = False
    while(not valid):
        print("Choose the arena:")
        print("Circular \nSquare")
        a_type = str(input())

        if a_type == "Square":
            valid = True
            write_square_arena()
            
            return 1
            
        elif a_type == "Circular":
            valid = True
            write_circular_arena()

            return 2

        else:
            print("No arena found of the type " + a_type)


if __name__ == '__main__':
    custom_maps_filepath: str = '../improved_maps/'

    print("Insert the map name: ")
    map_name = str(input())

    arena_type = choose_arena()

    resolution: float = 0.001
    max_pixel_value_for_wall: int = int(165.75) # possibly useless
    wall_pixels_coords: [(int, int)] = []


    height: int = map_size  # find out
    width: int = map_size

    # Create and save the new Webots file
    base_map_webots_filepath: str = custom_maps_filepath + 'base_map.wbt'
    f = open(base_map_webots_filepath, 'r')
    webots_str: str = f.read()
    f.close()

    map_webots_filepath: str = custom_maps_filepath + map_name + '.wbt'
    f = open(map_webots_filepath, 'w')
    f.write(webots_str)

    # f.write the shapes with their formats
    # save the wall coords at the same time

    f.close()


    # Create and save the coords file

    f = open(custom_maps_filepath + map_name + '_points.csv', 'w', newline='')
    writer = csv.writer(f)
    #    Add the borders
    for x in range(width):
        writer.writerow((resolution*x, 0))
        writer.writerow((resolution*x, resolution*(height - 1)))
    for y in range(1, height-1):
        writer.writerow((0, resolution*y))
        writer.writerow((resolution*(height - 1), resolution*y))  
    #    Add the walls
    for coord in wall_pixels_coords:
        writer.writerow(coord)
    f.close()


    # Create and save the map file

    # ORIGINAL WAY      - its the same thing for the shapes but with their formats
    index: int = 0
    for coord in wall_pixels_coords:
        f.write('Solid {')
        f.write('    translation ' + str(coord[0]) + ' ' + str(coord[1]) + ' 0.025')
        f.write('    children [')
        f.write('        Shape {')
        f.write('            geometry Box {')
        f.write('                size ' + str(resolution) + ' ' + str(resolution) + ' 0.05')
        f.write('            }')
        f.write('        }')
        f.write('    ]')
        f.write('    name "solid' + str(index) + '"')
        f.write('}')
        """
        Solid {
            translation -0.54 -0.06 0.025
            children [
                Shape {
                    geometry Box {
                        size 0.125 0.125 0.05
                    }
                }
            ]
            name "solid1"
        }
        """
        index += 1
    f.close()


# ADD MAP NAME (BEFORE ?)
# NAO ESQUECER DE FAZER POINTS.CSV A PARTIR DE COORDENADAS RELATIVAS DAS FIGURAS
# INICIAR ROBO ANTES(?) DE ESCREVER A ARENA