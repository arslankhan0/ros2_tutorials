# ROS 2 Minimal Publisher

This ROS 2 publisher code demonstrates several important C++ concepts. Here's an explanation of the key concepts used:

## The `MinimalPublisher` Class

The `MinimalPublisher` class is a ROS 2 node that publishes messages to a topic at regular intervals. Below is a detailed expl
anation of its components:

1. **Constructor**
   - The constructor initializes the node with the name `"minimal_publisher"`.
   - It creates a publisher for the `std_msgs::msg::String` message type on the topic `"topic"`.
   - It also sets up a timer to call the `timer_callback` function every 500 milliseconds.

2. **Timer Callback**
   - The `timer_callback` function is called by the timer at regular intervals.
   - It creates a message of type `std_msgs::msg::String`, sets its `data` field, and publishes it.
   - The `RCLCPP_INFO` function logs the message being published.

3. **Private Members**
   - `publisher_`: A shared pointer to the publisher object.
   - `timer_`: A shared pointer to the timer object.
   - `count_`: A counter to keep track of the number of messages published.

## Key C++ Concepts

1. **Classes and Objects**
   - The `MinimalPublisher` class encapsulates the functionality of the publisher node.
   - Objects of this class are created and managed in the `main()` function.

2. **Inheritance**
   - `MinimalPublisher` inherits from `rclcpp::Node`, which is the base class for all ROS 2 nodes.
   - This inheritance allows `MinimalPublisher` to use the functionality provided by `rclcpp::Node`, such as creating publishers and timers.

3. **Smart Pointers**
   - `std::shared_ptr`: Used to manage the lifetime of objects. For example:
     ```cpp
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     rclcpp::spin(std::make_shared<MinimalPublisher>());
     ```
   - Shared pointers ensure that the object is destroyed only when all references to it are gone.

4. **Lambdas and `std::bind`**
   - A lambda function is used in the `create_wall_timer` method:
     ```cpp
     std::bind(&MinimalPublisher::timer_callback, this)
     ```
   - `std::bind` binds the `timer_callback` method to the current instance (`this`) of the `MinimalPublisher` class. This allows the timer to call the member function.

5. **Timers**
   - The `create_wall_timer` method creates a timer that calls the `timer_callback` function at regular intervals (500ms in this case).
   - This is an example of asynchronous programming, where the timer runs independently of the main thread.

6. **ROS 2 Publisher**
   - The `create_publisher` method creates a publisher for the `std_msgs::msg::String` message type.
   - The `publish` method is used to send messages to the topic.

7. **Namespaces**
   - The `std` namespace is used for standard C++ features like `std::string`, `std::to_string`, and `std::bind`.
   - The `std::chrono_literals` namespace allows the use of time literals like `500ms`.

8. **Logging**
   - `RCLCPP_INFO` is used for logging messages. It is part of the ROS 2 logging system.
     Example:
     ```cpp
     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
     ```

9. **Encapsulation**
   - The `publisher_`, `timer_`, and `count_` members are private, ensuring that they are only accessible within the class.
   - This is a key principle of object-oriented programming.

10. **Main Function**
    - The `main()` function initializes the ROS 2 system with `rclcpp::init()`, creates an instance of the `MinimalPublisher` class, and spins the node to keep it alive.
    - `rclcpp::shutdown()` is called to cleanly shut down the ROS 2 system.

11. **Chrono Library**
    - The `<chrono>` library is used for time-related functionality.
    - `500ms` is a duration literal representing 500 milliseconds.

12. **String Manipulation**
    - `std::to_string(count_)` converts the integer `count_` to a string for concatenation with the message.

---

This code is a great example of combining modern C++ features with ROS 2 APIs to create a simple yet effective publisher