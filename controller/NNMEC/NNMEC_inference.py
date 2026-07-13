import torch
import torch.nn as nn
import numpy as np

class NNModel(nn.Module):
    """全結合NN (config: input_size, hidden_sizes, output_size, num_layers, activation_function)"""
    def __init__(self, input_size, hidden_sizes, output_size, num_layers=None, activation_function='relu'):
        super(NNModel, self).__init__()
        sizes = [input_size] + list(hidden_sizes) + [output_size]
        self.layers = sizes

        self.activation_function = self._get_activation(activation_function)

        # 学習時のクラスと同じ属性名 "linears" を使う(state_dictのキーを一致させるため)
        self.linears = nn.ModuleList(
            [nn.Linear(sizes[i], sizes[i + 1]) for i in range(len(sizes) - 1)]
        )

    @staticmethod
    def _get_activation(name):
        mapping = {
            'relu': nn.ReLU(),
            'tanh': nn.Tanh(),
            'sigmoid': nn.Sigmoid(),
            'leaky_relu': nn.LeakyReLU(),
            'silu': nn.SiLU(),
        }
        if isinstance(name, nn.Module):
            return name
        if isinstance(name, str):
            key = name.lower()
            if key not in mapping:
                raise ValueError(f"未対応の活性化関数です: {name}")
            return mapping[key]
        raise TypeError(f"activation_functionはstrまたはnn.Moduleである必要があります。実際の型: {type(name)}")

    def forward(self, x):
        """x: サイズは (input_size, N) を想定(学習時のforwardと同じ形式)"""
        for i, linear in enumerate(self.linears):
            x = linear(x)
            if i < len(self.linears) - 1:  # 出力層は恒等関数
                x = self.activation_function(x)
        return x
    

class RNNModel(nn.Module):
    """RNN (config: input_size, hidden_sizes, output_size, num_layers, activation_function)"""
    def __init__(self, input_size, hidden_sizes, output_size, num_layers, activation_function='relu'):
        super(RNNModel, self).__init__()
        # RNNは通常hidden_sizeは単一値だが、リストで来る可能性もあるため先頭を使用
        hidden_size = hidden_sizes[0] if isinstance(hidden_sizes, (list, tuple)) else hidden_sizes

        # nn.RNNのnonlinearityは'tanh'か'relu'のみ対応
        nonlinearity = activation_function.lower() if activation_function.lower() in ('tanh', 'relu') else 'relu'

        self.rnn = nn.RNN(input_size, hidden_size, num_layers,
                           batch_first=True, nonlinearity=nonlinearity)
        self.fc = nn.Linear(hidden_size, output_size)
        self.activation_function = self._get_activation(activation_function)

    def forward(self, x, h0=None):
        out, hn = self.rnn(x, h0)
        out = self.fc(out)
        return out, hn


class InferenceWrapper(nn.Module):
    def __init__(self, model_path):
        super(InferenceWrapper, self).__init__()
        self.device = torch.device("cpu") # 推論だけなのでcpuで十分
        checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
        config = checkpoint['config']

        if 'RNN' in model_path:
            self.model = RNNModel(**config)
        else:
            self.model = NNModel(**config)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval() # 推論モードに設定
        self.architecture_info = self._extract_info(config)

    def _extract_info(self, config):
        # configをそのまま返せば十分だが、見やすく整形
        return {
            'input_size': config['input_size'],
            'hidden_sizes': list(config['hidden_sizes']),
            'output_size': config['output_size'],
            'num_layers': config['num_layers'],
            'activation_function': config['activation_function'],
        }


    def predict(self, x_list):
        """NNModel / RNNModel共通の推論インターフェース"""
        with torch.no_grad():
            x = torch.tensor(x_list, dtype=torch.float32)
            # print(f"[DEBUG] input shape: {x.shape}, values: {x}") # inputの形状と値をMATLABコマンドウィンドウ上に出力

            if isinstance(self.model, RNNModel):
                if x.dim() == 2:
                    x = x.unsqueeze(0)  # (seq_len, input_size) -> (1, seq_len, input_size)
                out, _ = self.model(x)
                out = out.squeeze(0)
            else:
                out = self.model(x)
            # print(f"[DEBUG] output shape: {out.shape}, values: {out}") # outputの形状と値をMATLABコマンドウィンドウ上に出力

        return out.numpy().tolist()
    