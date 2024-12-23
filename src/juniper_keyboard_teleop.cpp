#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <SDL.h>
#include "juniper_board_msgs/srv/int_trigger.hpp"

using namespace std::chrono_literals;

class KeyboardTeleopNode : public rclcpp::Node
{
  public:
    KeyboardTeleopNode(): Node("juniper_keyboard_teleop")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_regulated", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardTeleopNode::timer_callback, this));

        this->declare_parameter<float>("linear_vel", 0.35);
        this->declare_parameter<float>("angular_vel", 2.5);
    
        // Initialize SDL, shutdown the node if SDL fails to initialize
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Could not initialize SDL: %s", SDL_GetError());
            rclcpp::shutdown();
        }

        // Set SDL to non-blocking mode
        SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

        SDL_WM_SetCaption("Juniper Keyboard Teleop", NULL);
        window = SDL_SetVideoMode(200, 200, 0, 0);  

        // Define the 9 squares as SDL_Rect
        int third_width = window->w / 3;
        int third_height = window->h / 3;

        // Define each square
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                squares[i * 3 + j] = {static_cast<Sint16>(j * third_width),static_cast<Sint16>(i * third_height), static_cast<Uint16>(third_width), static_cast<Uint16>(third_height)};
            }
        }


        client_ = this->create_client<juniper_board_msgs::srv::IntTrigger>("/actuate_head");


  
    }

    ~KeyboardTeleopNode() {
        SDL_FreeSurface(window);
        SDL_Quit(); // Clean up SDL on exit
    }

  private:

    SDL_Surface* window;
    SDL_Rect squares[9];
    int keyboard_array[5] = {}; //up, down, right, left, head state
    geometry_msgs::msg::Twist twist;
    int desired_head_state = 0;
    int current_head_state = 0;

    // Declare the service client as a private member
    rclcpp::Client<juniper_board_msgs::srv::IntTrigger>::SharedPtr client_;

    void timer_callback()
    {
        float linear_vel_x = this->get_parameter("linear_vel").get_value<float>();;
        float angular_vel_z = this->get_parameter("angular_vel").get_value<float>();;

        //Holds the sum of keyboard_array drive indicies (0-3)
        int drive_sum = 0;

        //The number of tries to wait for the /actuate_head service
        int service_try_count = 0;

        //When the SDL library catches an event on the keyboard
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            /*
            
            -Parsed which keys are pressed down, assigning a "1" in the keyboard_array indices 0-3 indicate moving
            -keyboard_array index 4 indicates the state of the head with possible values of 0-6
            -ESC will exit the node, CTRL+C will as well
            
            */ 
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_UP:
                        keyboard_array[0]=1;
                        break;
                    case SDLK_DOWN:
                        keyboard_array[1]=1;
                        break;
                    case SDLK_RIGHT:
                        keyboard_array[2]=1;
                        break;
                    case SDLK_LEFT:
                        keyboard_array[3]=1;
                        break;
                    case SDLK_0:
                        keyboard_array[4]=0;
                        break;         
                    case SDLK_1:
                        keyboard_array[4]=1;
                        break;
                    case SDLK_2:
                        keyboard_array[4]=2;
                        break;
                    case SDLK_3:
                        keyboard_array[4]=3;
                        break;
                    case SDLK_4:
                        keyboard_array[4]=4;
                        break;
                    case SDLK_5:
                        keyboard_array[4]=5;
                        break; 
                    case SDLK_6:
                        keyboard_array[4]=6;
                        break;              
                    case SDLK_ESCAPE:
                        RCLCPP_INFO(this->get_logger(), "Exiting Juniper Keyboard Teleop");
                        rclcpp::shutdown();
                        return;
                    default:
                        RCLCPP_INFO(this->get_logger(), "Invalid Key Selection: use the arrow keys to drive and numbers 0-6 to move the head");
                        break;
                }
            }

            // Check if a key was released, doesn't apply to actuating the head as those keys are not held down
            else if (event.type == SDL_KEYUP) {
                switch (event.key.keysym.sym) {
                    case SDLK_UP:
                        keyboard_array[0]=0;
                        break;
                    case SDLK_DOWN:
                        keyboard_array[1]=0;
                        break;
                    case SDLK_RIGHT:
                        keyboard_array[2]=0;
                        break;
                    case SDLK_LEFT:
                        keyboard_array[3]=0;
                        break;
                    default:
                        break;
                }
            }    

            //Parse forward and reverse
            if (keyboard_array[0]==1 && keyboard_array[1]==1){
                twist.linear.x=0;
                SDL_FillRect(window, &squares[1], SDL_MapRGB(window->format, 0, 0, 0));
                SDL_FillRect(window, &squares[7], SDL_MapRGB(window->format, 0, 0, 0));
                }
            else if (keyboard_array[0]==1){
                twist.linear.x=linear_vel_x;
                SDL_FillRect(window, &squares[1], SDL_MapRGB(window->format, 0, 255, 0));
                }
            else if (keyboard_array[1]==1){
                twist.linear.x=-linear_vel_x;
                SDL_FillRect(window, &squares[7], SDL_MapRGB(window->format, 255, 0, 0));
                }
            else{
                twist.linear.x=0.0;
                SDL_FillRect(window, &squares[1], SDL_MapRGB(window->format, 0, 0, 0));
                SDL_FillRect(window, &squares[7], SDL_MapRGB(window->format, 0, 0, 0));
                }
                
            //Parse left and right
             if (keyboard_array[2]==1 && keyboard_array[3]==1){
                twist.angular.z=0;
                SDL_FillRect(window, &squares[3], SDL_MapRGB(window->format, 0, 0, 0));
                SDL_FillRect(window, &squares[5], SDL_MapRGB(window->format, 0, 0, 0));
                }
            else if (keyboard_array[2]==1){
                twist.angular.z=-angular_vel_z;
                SDL_FillRect(window, &squares[5], SDL_MapRGB(window->format, 0, 0, 255));
                }
            else if (keyboard_array[3]==1){
                twist.angular.z=angular_vel_z;
                SDL_FillRect(window, &squares[3], SDL_MapRGB(window->format, 0, 0, 255));
                }
            else{
                twist.angular.z=0.0;
                SDL_FillRect(window, &squares[3], SDL_MapRGB(window->format, 0, 0, 0));
                SDL_FillRect(window, &squares[5], SDL_MapRGB(window->format, 0, 0, 0));
                }
            
        }
        //Publish the ROS Message
        publisher_->publish(twist);
        
        //Sum up the keyboard_array to check for drive keys
        for(int i = 0; i < 4 ; i++){
                drive_sum+=keyboard_array[i];
        }
        
        //Clear the window if not driving   
        if (drive_sum ==0 ){
            SDL_FillRect(window, NULL,SDL_MapRGB(window->format, 0, 0, 0)); 
        }     
        
        //Update the window
        SDL_Flip(window);  

        //Grab the desired head state from the keyboard_array
        desired_head_state = keyboard_array[4];
        
        //Only call the service when changing head states
        if (desired_head_state != current_head_state ){

            while (!client_->wait_for_service(1s)) {
                if (service_try_count >= 3){
                    RCLCPP_INFO(this->get_logger(), "Failed: /actuate_head service is not available...");
                    break;
                }
                if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the /actuate_head service. Exiting.");
                        return;
                } 
                RCLCPP_INFO(this->get_logger(), "Waiting for the /actuate_head service to be available...Attempt %d/3", service_try_count+1);
                service_try_count++;
            }

            // Create a request for the actuate_head
            auto request = std::make_shared<juniper_board_msgs::srv::IntTrigger::Request>();
            request->data = desired_head_state;

            // Move the head to the desired state
            client_->async_send_request(request, 
                [this](rclcpp::Client<juniper_board_msgs::srv::IntTrigger>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "/actuate_head service call successful: Position %d", desired_head_state);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "/actuate_head service call failed:");
                    }
                });
            //Update the current state of the head
            current_head_state = desired_head_state;
        }

        
    
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
  return 0;
}