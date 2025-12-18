import json
import matplotlib.pyplot as plt
import numpy as np

def plot_filtered_telemetries(json_file):
    # Carregar o arquivo JSON
    with open(json_file, 'r') as f:
        data = json.load(f)

    if 'telemetries' not in data:
        print("Chave 'telemetries' não encontrada.")
        return

    # Lista de variáveis a serem ignoradas
    exclude_keys = ['dutyCycle%', 'dutyCycle', 'u']
    
    telemetries = data['telemetries']
    plot_data = {}
    all_timestamps = []

    # Extrair e filtrar dados
    for name, content in telemetries.items():
        if name in exclude_keys:
            continue
            
        if isinstance(content, dict) and 'data' in content and len(content['data']) == 2:
            timestamps_raw = content['data'][0]
            values_raw = content['data'][1]
            
            ts = []
            vs = []
            # Filtrar valores None e timestamps inválidos
            for t, v in zip(timestamps_raw, values_raw):
                if t is not None and v is not None and t > 1000:
                    ts.append(t)
                    vs.append(v)
            
            if ts:
                plot_data[name] = {'t': ts, 'v': vs}
                all_timestamps.extend(ts)

    if not plot_data:
        print("Nenhum dado restante para plotar.")
        return

    # Definir tempo inicial global para normalizar o eixo X
    start_time = min(all_timestamps)

    # Gerar um gráfico separado para cada variável restante
    for name, d in plot_data.items():
        t = np.array(d['t'])
        v = np.array(d['v'])
        
        # Tempo relativo em segundos
        t_rel = t - start_time
        
        plt.figure(figsize=(10, 4))
        plt.plot(t_rel, v, label=name, color='tab:blue')
        plt.title(f"Gráfico: {name}")
        plt.xlabel(f"Tempo (s) desde {start_time}")
        plt.ylabel(name)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.tight_layout()
        
        # Salvar o gráfico
        filename = f"plot_{name}.png"
        plt.savefig(filename)
        print(f"Gráfico salvo: {filename}")
        plt.show()

# Executar a função
plot_filtered_telemetries('/home/dino/Documents/PlatformIO/Projects/DUNK/src/proporcioanl.json')