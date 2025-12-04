import matplotlib.pyplot as plt
import re

# Nome do arquivo de entrada
nome_arquivo = 'entradas.txt'

# Listas para armazenar os dados
pitch_filtrado = []
pitch_bruto = []
amostras = [] # Eixo X (contador)

print(f"Lendo arquivo: {nome_arquivo}...")

try:
    with open(nome_arquivo, 'r') as arquivo:
        contador = 0
        for linha in arquivo:
            # Expressão regular para capturar os números (incluindo negativos e decimais)
            # Procura por "Pitch_Filtrado:" seguido de número e "Pitch_Bruto:" seguido de número
            match = re.search(r'Pitch_Filtrado:([-+]?\d*\.?\d+).*Pitch_Bruto:([-+]?\d*\.?\d+)', linha)
            
            if match:
                try:
                    val_filtrado = float(match.group(1))
                    val_bruto = float(match.group(2))
                    
                    pitch_filtrado.append(val_filtrado)
                    pitch_bruto.append(val_bruto)
                    amostras.append(contador)
                    contador += 1
                except ValueError:
                    continue # Pula linha se houver erro na conversão

    print(f"Total de linhas lidas com sucesso: {len(amostras)}")

    if len(amostras) == 0:
        print("Nenhum dado encontrado! Verifique se o arquivo txt está no formato correto.")
        exit()

    # ==========================================
    # CRIAÇÃO DOS GRÁFICOS (2 SUBPLOTS)
    # ==========================================
    
    # Cria uma figura e dois eixos (ax1 em cima, ax2 embaixo)
    # sharex=True faz com que ao dar zoom em um, o outro acompanhe
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Gráfico 1: Pitch Filtrado (Kalman)
    ax1.plot(amostras, pitch_filtrado, color='blue', linewidth=1.5, label='Filtrado (Kalman)')
    ax1.set_title('Pitch Filtrado (Kalman)', fontsize=14)
    ax1.set_ylabel('Graus (°)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper right')

    # Gráfico 2: Pitch Bruto (Sem Filtro)
    ax2.plot(amostras, pitch_bruto, color='red', linewidth=1.0, alpha=0.7, label='Bruto (Sem Filtro)')
    ax2.set_title('Pitch Bruto (Acelerômetro)', fontsize=14)
    ax2.set_xlabel('Amostras', fontsize=12)
    ax2.set_ylabel('Graus (°)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper right')

    plt.tight_layout() # Ajusta o espaçamento para não sobrepor textos
    plt.show()

except FileNotFoundError:
    print(f"ERRO: O arquivo '{nome_arquivo}' não foi encontrado.")
except Exception as e:
    print(f"Ocorreu um erro inesperado: {e}")