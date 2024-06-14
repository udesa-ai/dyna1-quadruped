### Proceso de Iniciación

A seguir se presenta el paso a paso para inicializar el cuadrúpedo.

#### Paso 1

Corra el código de bash de "can_setup.sh" para activar el puerto CAN de la TX2:

```
./can_setup.sh
```

#### Paso 2

Reinicie los odrive con los motores en las posiciones que utilizará de referencia para cada inicialización.

```
python3 reboot.py
```

#### Paso 3

Una vez reiniciadas y orientadas las piernas corra el código "firmware_workaround.py" para activar el sensado de ángulo. 

```
python3 firmware_workaround.py
```

#### Paso 4

Si nunca calibró las posiciones de los encoder con la de las piernas corra el código de python "calibrate.py". A medida que le pregunte si está listo para calibrar cada pierna, posiciones las en el punto de referencia que decida utilizar.

```
python3 calibrate.py
```

#### Paso 5

Ahora con el robot calibrado y puesto en punto puede abrir el docker de Ubuntu 22 del Dockerfile y ahi correr ROS2 con los launch files.