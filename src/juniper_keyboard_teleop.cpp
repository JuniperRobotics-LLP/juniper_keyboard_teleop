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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class KeyboardTeleopNode : public rclcpp::Node
{
  public:
    KeyboardTeleopNode(): Node("juniper_keyboard_teleop")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardTeleopNode::timer_callback, this));
    
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
                squares[i * 3 + j] = {j * third_width, i * third_height, third_width, third_height};
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
    float linear_vel_x = 0.35;
    float angular_vel_z = 2.5;
    int desired_head_state = 0;
    int current_head_state = 0;

    // Declare the service client as a private member
    rclcpp::Client<juniper_board_msgs::srv::IntTrigger>::SharedPtr client_;

    void timer_callback()
    {
        //Holds the sum of keyboard_array drive indicies (0-3)
        int drive_sum = 0;

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_KEYDOWN) {
                // RCLCPP_INFO(get_logger(), "KeyDown %d", event.key.keysym.sym);
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
                        RCLCPP_INFO(this->get_logger(), "Invalid Key Selection");
                        break;
                }
            }

            else if (event.type == SDL_KEYUP) {
                // RCLCPP_INFO(get_logger(), "KeyUp %d", event.key.keysym.sym);
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

        /* Change the color of the window based on if any drive key is still pressed */
        
        //Sum up the keyboard_array to check for drive keys
        for(int i = 0; i < 4 ; i++){
                drive_sum+=keyboard_array[i];
        }
        
        //Green if a moving
        if (drive_sum !=0 ){
            SDL_Flip(window);  
        }
        //Red if stopped        
        else{
            SDL_FillRect(window, NULL,SDL_MapRGB(window->format, 0, 0, 0));  
            SDL_Flip(window);  
        }
  
        desired_head_state = keyboard_array[4];
        
        //Only call the service when changing head states
        if (desired_head_state != current_head_state ){

            while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                } 
            RCLCPP_INFO(this->get_logger(), "Waiting for the custom service to be available...");
            }

            // Create a request for the custom service
            auto request = std::make_shared<juniper_board_msgs::srv::IntTrigger::Request>();
            request->data = desired_head_state;

            // Asynchronous service call
            client_->async_send_request(request, 
                [this](rclcpp::Client<juniper_board_msgs::srv::IntTrigger>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Custom service call successful");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Custom service call failed:");
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