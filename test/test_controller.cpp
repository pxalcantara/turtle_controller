#include "gtest/gtest.h"
#include "controller_server.hpp"

class ControllerTest : public ::testing::Test {
protected:
    std::shared_ptr<Controller> controller;
    std::vector<float> range;
    float sector_width = 20.0f * (M_PI / 180.0f);

    static void SetUpTestSuite() {
        // Inicializando o contexto ROS2
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        // Encerrando o contexto ROS2 após os testes
        rclcpp::shutdown();
    }

    void SetUp() override {
        // Criando uma instância do Controller
        controller = std::make_shared<Controller>();

        // Criando e configurando os parâmetros do LaserScanInfo
        LaserScanInfo laser_parameters;
        laser_parameters.angle_min = -M_PI;
        laser_parameters.angle_max = M_PI;
        laser_parameters.angle_increment = M_PI / 180.0;  // 1 grau

        // Configurando os parâmetros na instância do Controller
        controller->set_laser_info(laser_parameters);

        // Definindo um vetor de ranges fictícios
        for (int i = 0; i < 360; ++i) {
            range.push_back(static_cast<float>(i));  // Preenchendo com índices fictícios
        }
    }
};
// Exemplo de um teste unitário
TEST_F(ControllerTest, LinearVelocity) {
    // auto node = std::make_shared<Controller>();
    
    ASSERT_FLOAT_EQ(0.2, controller->get_linear_velocity());
    controller->set_linear_velocity(1.7);
    ASSERT_FLOAT_EQ(1.7, controller->get_linear_velocity());
}

TEST_F(ControllerTest, GetFrontSectorRange) {
    // Centro do setor: 0 radianos (à frente do robô)
    float sector_center_position = 0.0f; 
    // float sector_center_position = 90.0f * (M_PI / 180.0f); 

    // Largura do setor: 20 graus = 20 * (π/180) radianos
    // float sector_width = 20.0f * (M_PI / 180.0f);

    // Executando a função
    std::vector<float> result = controller->get_sector_range(sector_center_position, sector_width, range);
    LaserScanInfo laser_info = controller->get_laser_info();

    // Calculando os índices esperados
    int start_index = static_cast<int>((sector_center_position - sector_width / 2 - laser_info.angle_min) / laser_info.angle_increment);
    int end_index = static_cast<int>((sector_center_position + sector_width / 2 - laser_info.angle_min) / laser_info.angle_increment);

    // Validando se o range retornado corresponde ao setor
    for (int i = start_index; i <= end_index; ++i) {
        // std::cout << "Índice: " << i << ", Valor: " << range[i] << std::endl;

        EXPECT_EQ(result[i - start_index], range[i]) << "Erro no índice " << i;
    }

    // Verificando o tamanho do vetor de resultados
    EXPECT_EQ(result.size(), end_index - start_index + 1) << "Tamanho do vetor de range incorreto.";
    ASSERT_FLOAT_EQ(180, controller->get_sector_range_mean(result));

}