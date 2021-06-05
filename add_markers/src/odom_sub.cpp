#include <nav_msgs/Odometry.h> // Necessário para acessar dados de Odometria
#include <ros/ros.h> // Necessário para criar o nó, etc

// Estas funções são chamadas quando uma nova mensagem de odometria chega ao assinante
// Recebe automaticamente a mensagem odometria como um parâmetro
// Imprime várias partes da mensagem
void counterCallback (const nav_msgs :: Odometry :: ConstPtr & msg) {
  // ROS_INFO ("% s", msg-> header.frame_id.c_str ());
  // ROS_INFO ("% f", msg-> twist.twist.linear.x);
  ROS_INFO ("%f", msg-> pose.pose.position.x);
}

int main (int argc, char ** argv) {
  // Crie um nó para executar o código
  ros :: init (argc, argv, "odom_sub_node");
  ros :: NodeHandle nh;

  // cria um assinante para o tópico "/ odom" para que possamos obter a mensagem odometria
  ros :: Subscriber sub = nh.subscribe ("odom", 1000, counterCallback);
  
  // Execute o programa até que seja parado manualmente
  ros :: spin ();

  return 0;
}
