import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohsenium/Torpedo/spawn_turtle_game/install/turtle_game_controller'
