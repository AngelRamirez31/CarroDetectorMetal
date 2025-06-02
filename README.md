**Explorador Metálico** es un robot autónomo diseñado para detectar y avisar sobre objetos metálicos en entornos complejos (terrenos irregulares, playas, áreas con escombros, etc.). 
Emplea un microcontrolador STM32 Nucleo-F401RE, un detector de metales MDS-60, un sensor ultrasónico HC-SR04 y un sensor KY-037 para emitir un zumbido al detectar metal. 
Su chasis con ruedas y controladores L298N le permiten moverse, retroceder y girar para evitar obstáculos.

El **Explorador Metálico** es una plataforma robótica autónoma cuyo propósito es:

- Patrollear áreas amplias sin intervención humana.
- Detectar metales (enter­rados o a la vista) mediante un detector MDS-60 que emite un zumbido al aproximarse a un objeto metálico.
- Evitar obstáculos (rocas, ramas, etc.) usando un sensor ultrasónico HC-SR04.
- Informar al usuario mediante un pequeño LCD y, adicionalmente, por UART si se ha detectado metal (o ruido) con el sensor KY-037.
- Mostrar en tiempo real la distancia medida por el HC-SR04 en pantalla.

Este proyecto sirve tanto para aplicaciones de limpieza ambiental (recolección de fragmentos metálicos) como para fines educativos y de aprendizaje en robótica embebida.
