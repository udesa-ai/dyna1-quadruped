from classes.motors import Motors

motores = Motors()

pos = dict.fromkeys(motores.names,None)

print('############################################################')
input('ready?')

motores.turn_on(pos)

motores.turn_off(pos)

print('Done!')
