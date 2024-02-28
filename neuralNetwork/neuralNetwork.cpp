#include <torch/script.h> // One-stop header.
#include <iostream>
#include <vector>
#include <memory>

class NeuralNet {
public:
    NeuralNet(const std::string& model_path){
        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            module = torch::jit::load(model_path);
            std::cout << "Neural network successfully loaded.\n";
        } catch (const c10::Error& e) {
            std::cerr << "error loading the model\n";
            throw;
        }
    }

    std::vector<float> forward(const std::vector<torch::jit::IValue>& inputs) {
        // Execute the model and get the output tensor
        torch::Tensor output_tensor = module.forward(inputs).toTensor();

        // Convert the output tensor to a std::vector<float>
        return tensorToVector(output_tensor);
    }

private:
    torch::jit::script::Module module;

    std::vector<float> tensorToVector(const torch::Tensor& tensor) {
        // Get the number of elements in the tensor
        size_t numElements = tensor.numel();

        // Create a vector to hold the tensor elements
        std::vector<float> vec(numElements);

        // Copy the tensor data to the vector
        memcpy(vec.data(), tensor.data_ptr<float>(), numElements * sizeof(float));

        return vec;
    }
};

std::vector<float> compute() {
  auto net = NeuralNet("/home/mwaltz/python_to_cpp/traced_mlp.pt");

  // Create a vector of inputs.
  std::vector<torch::jit::IValue> inputs;

  // Manually fill the input tensor with specific numbers.
  torch::Tensor input_tensor = torch::tensor({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});

  // Add the input tensor to the vector of inputs.
  inputs.push_back(input_tensor);

  // Execute the model and turn its output into a tensor.
  std::vector<float> output = net.forward(inputs);
  return output;
}

int main(){

}
