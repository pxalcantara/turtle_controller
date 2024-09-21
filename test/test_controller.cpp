#include "gtest/gtest.h"
#include "controller_server.hpp"

// Teste de exemplo para a classe ControllerServer
class ControllerServerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        // Inicializa o contexto do ROS2 antes de executar os testes
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        // Encerra o contexto do ROS2 após os testes
        rclcpp::shutdown();
    }
};

// Exemplo de um teste unitário
TEST_F(ControllerServerTest, LinearVelocity) {
    auto node = std::make_shared<Controller>();
    
    ASSERT_FLOAT_EQ(0.2, node->get_linear_velocity());
    node->set_linear_velocity(1.7);
    ASSERT_FLOAT_EQ(1.7, node->get_linear_velocity());
}