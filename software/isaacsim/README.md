## 🦾 Configuración del entorno para Dyna1 en Isaac Sim

### Isaac Lab
**Isaac Lab** es una librería construida sobre Isaac Sim que facilita el desarrollo de entornos de simulación para 
aprendizaje por refuerzo (RL), control óptimo y robótica en general. Se basa en gymnasium, y ofrece herramientas ya integradas para:

- Definir entornos personalizados tipo Gym.
- Entrenar políticas con algoritmos populares como PPO.
- Simular robots con interfaces limpias y separadas.
- Integrarse con librerías de RL como RL-Games, Stable-Baselines3, etc.

#### ¿Por qué usarlo?
- Estandariza cómo definís los entornos, sensores, recompensas y acciones.
- Aísla tu entorno personalizado del resto de Isaac Sim → más fácil de mantener y migrar.
- Ya tiene soporte para entrenamiento distribuido y acelerado por GPU.
- Facilita la transferencia del modelo a un robot real (sim2real).

### Isaac Extension Template
Es una plantilla modular creada para estructurar proyectos en Isaac Sim de forma escalable y mantenible:

Incluye:
- Separación clara entre código, assets, configuraciones.
- Soporte para definir extensiones de OmniIsaacGymEnvs, robots, tareas, sensores, etc.
- Integración con Isaac Lab out-of-the-box.

#### ¿Por qué usarlo?
- Permite trabajar en equipo con una arquitectura limpia y modular.
- Podés agregar o quitar componentes sin romper todo el proyecto.
- Si necesitás escalar, migrar o versionar tu entorno, esto evita el caos.


1. Clonar los repositorios necesarios
    
	    cd ~
	    https://github.com/isaac-sim/IsaacLabExtensionTemplate.git
	    https://github.com/isaac-sim/IsaacLab.git

2. Mover el entorno del Dyna1

    	mv dyna1-quadruped/software/isaacsim/DynabotIsaacTemplate IsaacLabExtensionTemplate/source

**DynabotIsaacTemplate** contiene toda la configuración específica del robot Dyna1 aislada de Isaac Lab. Esto significa que cualquier cambio en Isaac Lab no debería afectar nuestro entorno (en principio), manteniendo todo modular y desacoplado.

3. Editar el archivo en **IsaacLab/docker/docker-compose.yaml** y agregar estas líneas dentro de la sección binds del servicio (al final de esa lista):

		- type: bind
		source: /home/$USER/IsaacLabExtensionTemplate
		target: /workspace/IsaacLabExtensionTemplate

¿Qué hace esto?

- **type: bind:** Monta un volumen directamente desde el sistema de archivos del host.
- **source:** Ruta local donde está el proyecto (reemplazá $USER por tu nombre de usuario si no funciona).
- **target:** Ruta dentro del contenedor donde se verá esa carpeta.

4.  Iniciar el contenedor
   
   		./container.sh start


Es posible que la primera vez te aparezca este mensaje (ver Troubleshooting):

    X11 Forwarding is configured as '0' in '.container.cfg'.
	To enable X11 forwarding, set 'X11_FORWARDING_ENABLED=1' in '.container.cfg'.

5. Ingresar al contenedor

		./container.sh enter base

y una vez dentro del docker hacer:

    cd /workspace/IsaacLabExtensionTemplate/

6. Instalar todas las dependencias del Dyna1:

   	python -m pip install -e source/DynabotIsaacTemplate

Si falla el paso anterior es poque probablemente antes haya que actualizar pip a la ultima version:

    /workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install --upgrade pip

7. Si se quire correr la simulación en una máquina y visualizarla desde otra, exportá esta variable:

    	export LIVESTREAM=2

Esto es por si se esta usando una PC para correr el entrenamiento y se quiere visualizar el output de la simulacion desde otra.

8. Modificar import en script de entrenamiento

Editar la línea 68 del archivo IsaacLabExtensionTemplate/scripts/rsl_rl/train.py:

Antes:

	import ext_template.tasks  # noqa: F401

Después:
    
    import DynabotIsaacTemplate.tasks  # noqa: F401

9. Correr la simulación del Dyna1

Si todo fue bien (aunque probablemente algo haya que ajustar), lanzar la simulación con:

    python scripts/rsl_rl/train.py --task=Dyna1-Flat-v0 --headless --num_envs=32

Este comando indica que se va a lanzar la simulacion con terrenos **flat**, de manera **headless** con 32 robots. 

## Troubleshooting

- Si la simulación no se lanza en headless desde docker, puede ser que esté intentando hacer forward del GUI y crashee (porque tenés deshabilitado x11).
- Configurar esto en **isaacLab/docker/.container.cfg**:
  
      [X11]
      x11_forwarding_enabled = 0




     



    
