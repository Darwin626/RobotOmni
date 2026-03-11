#include <micro_ros_arduino.h>      //Importar la libreria de micro-ros
#include <stdio.h>                  //Importar la libreria de stdio
#include <rcl/rcl.h>                //Importar la libreria de rcl
#include <rcl/error_handling.h>     //Importar la libreria de rcl error handling
#include <rclc/rclc.h>              //Importar la libreria de rclc
#include <rclc/executor.h>          //Importar la libreria de rclc executor
#include <std_msgs/msg/float32.h>   //Importar la libreria de std_msgs

rcl_subscription_t subscriber;      //Declarar el subscriber
rcl_publisher_t publisher;          //Declarar el publisher
std_msgs__msg__Float32 msg_vel1;    //Declarar el mensaje de entrada de tipo Float32
rclc_executor_t executor;           //Declarar el executor que se encargara de ejecutar el subscriber
rclc_support_t support;             //Declarar el support que se encargara de inicializar el micro-ros
rcl_allocator_t allocator;          //Declarar el allocator que se encargara de asignar memoria
rcl_node_t node;                    //Declarar el nodo que se encargara de enviar el mensaje
rcl_timer_t timer;                  //Declarar el timer que se encargara de enviar el mensaje


#define LED_PIN 2                   //Definir el pin del led

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}   //Definir la macro RCCHECK que se encargara de verificar si la funcion devuelve un error
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}            //Definir la macro RCSOFTCHECK que se encargara de verificar si la funcion devuelve un error

//Funcion que se encargara de reiniciar el ESP32 en caso de error
void error_loop(){
  for(int i=10; i>0; i--){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  ESP.restart();
}

//Definir la funcion que se encargara de recibir el mensaje
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float v_ref = msg->data;                                                        //Definir el valor del mensaje a recibir
  //
  //---ALGORITMO DE CONTROL PID---
  //  
}

//Definir la funcion que se encargara de enviar el mensaje
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);                                                      //Definir el valor de last_call_time
  if(timer != NULL){
    digitalWrite(LED_PIN, HIGH);                                                    //Encender el led
    RCCHECK(rcl_publish(&publisher, &msg_vel1, NULL));                              //Enviar el mensaje
    msg_vel1.data++;                                                                //Definir el valor del mensaje a enviar
    //msg_vel1.data = lectura encoder
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.begin(921600);
  set_microros_transports();                                                      //Configurar los transportes de micro-ros
  pinMode(LED_PIN, OUTPUT);                                                       //Configurar el pin del led como salida
  digitalWrite(LED_PIN, HIGH);                                                    //Encender el led

  delay(2000);                                                                    //Esperar 2 segundos
  digitalWrite(LED_PIN, LOW);

  allocator = rcl_get_default_allocator();                                        //Definir el allocator que se encargara de asignar memoria

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));                      //Crear el support que se encargara de inicializar el micro-ros

  // create node
  RCCHECK(rclc_node_init_default(&node, "Nodo_motor1", "", &support));            //Crear el nodo que se encargara de enviar el mensaje

  // create subscriber
  RCCHECK(rclc_subscription_init_default(                                         //Crear el subscriber
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),                          //Definir el tipo de mensaje que se va a recibir
    "cmd_vel1"));                                                                 //Definir el nombre del topic al que se va a suscribir

  // create publisher
  RCCHECK(rclc_publisher_init_default(                                            //Crear el publisher de tipo reliable
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),                          //Definir el tipo de mensaje que se va a enviar
    "vel1"));                                                                     //Definir el nombre del topic al que se va a publicar

  // create timer
  const unsigned int timer_timeout = 100;                                         //Definir el tiempo de espera del timer
  RCCHECK(rclc_timer_init_default(                                                //Crear el timer
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),                                                  //Definir el tiempo de espera del timer
    timer_callback));                                                             //Definir la funcion que se va a ejecutar

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));        //Crear el executor que se encargara de ejecutar el subscriber
  RCCHECK(rclc_executor_add_timer(&executor, &timer));                            //Agregar el timer al executor
  msg_vel1.data = 0.0;
}

//Funcion que se encargara de ejecutar el subscriber
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));             //Ejecutar el executor
}