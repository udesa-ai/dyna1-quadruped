## Seteo del ambiente Dyna1 usando Isaaclab y IsaacLabExtensionTemplate.

Parados desde su home los siguientes comando son para confugurar el ambiente:

    cd

Se deberian clonar **IsaacLabExtensionTemplate** y **IsaacLab**:

    https://github.com/isaac-sim/IsaacLabExtensionTemplate.git
    https://github.com/isaac-sim/IsaacLab.git

Mover **DynabotIsaacTemplate** a **IsaacLabExtensionTemplate/source**:

    mv dyna1-quadruped/software/isaacsim/DynabotIsaacTemplate IsaacLabExtensionTemplate/source

**DynabotIsaacTemplate** contiene toda la configuracion del Dyna1 aislada de Isaaclab, cualquier cambio que se haga en Isaaclab no afectaria (en principio a nuestro) ambiente.

Editar el archivo **IsaacLab/docker/docker-compose.yaml** agregarle las siguiente lineas al final de la configuracion de **bind**

    - type: bind
    source: /home/$USER/IsaacLabExtensionTemplate
    target: /workspace/IsaacLabExtensionTemplate

	type: bind
Indica que el volumen es un montaje tipo “bind”, es decir, se vincula directamente un directorio del host al contenedor.

	source: /home/$USER/IsaacLabExtensionTemplate
Es la ruta en tu máquina local (host) que quieres montar. Aquí, $USER se reemplaza por tu nombre de usuario, y apunta a la carpeta del proyecto en tu home.

	target: /workspace/IsaacLabExtensionTemplate
Es la ruta dentro del contenedor donde ese mismo directorio aparecerá. Cualquier archivo o cambio en /workspace/IsaacLabExtensionTemplate dentro del contenedor afecta directamente al source en el host.



Ejecutar el docker compose con el siguiente comando:

    ./container.sh start

Este comando la primera vez puede que pida la siguiente configuracion:

    X11 Forwarding is configured as '0' in '.container.cfg'.
	To enable X11 forwarding, set 'X11_FORWARDING_ENABLED=1' in '.container.cfg'.

Una vez compilado el docker ejecutar:

    ./container.sh enter base

Dentro del docker hacer:

    cd /workspace/IsaacLabExtensionTemplate/

Instalar todas las dependencias del Dyna1:

     python -m pip install -e source/DynabotIsaacTemplate

Si falla el paso anterior es poque probablemente antes haya que actualizar pip a la ultima version:

    /workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install --upgrade pip

Para hacer stream a otra PC (o estando desde docker a la pc host) se deberia correr este comando: 
    
    export LIVESTREAM=2

Esto es por si se esta usando una PC para correr el entrenamiento y se quiere visualizar el output de la simulacion desde otra.

Modificar la linea 68 de:

    68:import ext_template.tasks  # noqa: F401

por:
    
    68:import DynabotIsaacTemplate.tasks  # noqa: F401

finalmente si todo salio bien (aunque es muy probable que no) con el siguiente comando se deberia poder correr la simulacion para el Dyna1:

    python scripts/rsl_rl/train.py --task=Dyna1-Flat-v0 --headless --num_envs=32

Este comando indica que se va a lanzar la simulacion con terrenos **flat**, de manera **headless** con 32 robots. 

## Troubleshooting

- Si no se lanza la simulacion de manera **headless** desde docker, este intenta hacer un forward del output de la simulacion y crashea porque tenemos desabilitado **x11**
- Se puede dejar seteada una configuracion para x11 desde el archivo en **IsaacLab/docker/.container.cfg**:
  
      [X11]
      x11_forwarding_enabled = 0




     



    
