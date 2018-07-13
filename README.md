
## Codificador de Referência HM.16 (HEVC) Otimizado por meio de Aprendizado de Máquina no Mecanismo de Particionamento do Codificador

O codificador de vídeo HEVC/H.265 apresenta um aumento significativo de eficiência de compressão em relação ao seu antecessor, o AVC/H.264. Isto é atribuído ao mecanismo de particionamento de quadros utilizado pelo HEVC, que oferece uma estrutura mais flexível de divisão dos blocos. Inerente a esse aumento na flexibilidade, o aumento de esforço computacional necessário para compressão representa desafios que devem ser enfrentados. Com o objetivo de reduzir o custo computacional do codificador HEVC, neste trabalho foi proposto o uso de aprendizado de máquina para predizer a forma de particionamento dos quadros durante a codificação. Foi utilizado um algoritmo de aprendizado de máquina em tempo real, no codificador de referência HM16.7. Com essa abordagem foi possível obter uma redução de 40\%, em média, no tempo de processamento, mantendo a qualidade do vídeo resultante semelhante à obtida com o codificador de referência.

## Para usar:

Para usar este trabalho, use a devida referência:

	@INPROCEEDINGS{raimundo:18, 
		author={José Raimundo Barbosa and Jean Felipe Felipe de Oliveira and Ruan Delgado Gomes and Carlos Danilo Miranda Regis e Marcelo Sampaio de Alencar}, 
		booktitle={XXXVI SIMPÓSIO BRASILEIRO DE TELECOMUNICAÇÕES E PROCESSAMENTO DE SINAIS}, 
		title={Uso de Aprendizado de Máquina no Mecanismo de Particionamento do Codificador de Vídeo HEVC}, 
		year={2018}, 
		volume={1}, 
		keywords={HEVC, Machine learning, video coding}, 
	}