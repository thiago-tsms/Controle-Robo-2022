#ifndef Kalman_H
#define Kalman_H


class Kalman{
  
	private:
			/* --- Variáveis para filtro de kalman --- */
		float x[2];
		float p[2][2];
		float q[2][2];
		float r[2][2];
	
	
	  /* --- FUNÇÕES --- */

	public: Kalman(){
	x[0] = 0;   //Angulo 
	x[1] = 0;   //Escorregamento do giroscópio

		//Covariancia (Esta sendo inicialidade por problemas de divisão por 0 em algum ponto)
	p[0][0] = 0.0000001;
	p[0][1] = 0; 
	p[1][0] = 0;
	p[1][1] = 0.0000001;  

		//Covariancia do acelerômetro
	q[0][0] = /*0.0029247451;*/  0.01; //Precisão
	q[0][1] = 0;
	q[1][0] = 0;
	q[1][1] = /*0.0219726563;*/ 0.000219726563; /*0.0000219726563;*/   //Escorregamento

		//Desvio padão da medição
	r[0][0] = 0.010785424672300061; /*0.008379820652313663; */
	r[0][1] = 0;
	r[1][0] = 0;
	r[1][1] = /*0.0000534057617;*/ /*0.000534057617;*/ 0.00534057617;
	}


	//Função que realiza o processo de filtragem
	public: float filtro(float acel, float giro, float dt){
    /*
      acel - angulo calculado do acelerômetro (°)
      giro - valor lido pelo giroscópio (°/seg)
      dt - tempo entre as medições (seg)
    */

      //Variáveis auxiliares
    float p_k[2][2], k[2][2], k_aux[2][2], z[2], z_k[2];

    z[0] = acel;
    z[1] = 0;

      //Estima o próximo estado
    x[0] += (giro - x[1]) * dt;

      //Estima a covariancia do erro
    p_k[0][0] = p[0][0] - ((p[1][0] + p[0][1]) * dt) + (p[1][1] * (dt * dt)) + (q[0][0] * dt);
    p_k[0][1] = p[0][1] - (p[1][1] * dt) + (q[0][1] * dt);
    p_k[1][0] = p[1][0] - (p[1][1] * dt) + (q[1][0] * dt);
    p_k[1][1] = p[1][1] + (q[1][1] * dt);

      //Atualiza o ganho
    k[0][0] = p_k[0][0] + r[0][0];
    k[0][1] = p_k[0][1] + r[0][1];
    k[1][0] = p_k[1][0] + r[1][0];
    k[1][1] = p_k[1][1] + r[1][1];
      //Calcula a Inversa (Passo para divisão das matrizes)
    k_aux[0][0] = k[1][1] / ((k[0][0] * k[1][1]) - (k[0][1] * k[1][0]));
    k_aux[0][1] = k[0][1] / ((k[0][1] * k[1][0]) - (k[0][0] * k[1][1]));
    k_aux[1][0] = -(k[1][0] * k_aux[0][0]) / k[1][1];
    k_aux[1][1] = -(k[0][0] * k_aux[0][1]) / k[0][1];
      //Multiplica as Matrizes (Passo para divisão das matrizes)
    k[0][0] = (p_k[0][0] * k_aux[0][0]) + (p_k[0][1] * k_aux[1][0]);
    k[0][1] = (p_k[0][0] * k_aux[0][1]) + (p_k[0][1] * k_aux[1][1]);
    k[1][0] = (p_k[1][0] * k_aux[0][0]) + (p_k[1][1] * k_aux[1][0]);
    k[1][1] = (p_k[1][0] * k_aux[0][1]) + (p_k[1][1] * k_aux[1][1]);

      //Atualiza a estimativa com a medição
    z_k[0] = z[0] - x[0];
    z_k[1] = z[1] - x[1];
    x[0] += (k[0][0] *  z_k[0]) + (k[0][1] *  z_k[1]);
    x[1] += (k[1][0] *  z_k[0]) + (k[1][1] *  z_k[1]);

      //Atualiza a covariancia do erro
    p[0][0] = ((1 - k[0][0]) * p_k[0][0]) + (k[0][1] * p_k[1][0]);
    p[0][1] = ((1 - k[0][0]) * p_k[0][1]) + (k[0][1] * p_k[1][1]);
    p[1][0] = (k[1][0] * p_k[0][0]) + ((1 - k[1][1]) * p_k[0][1]);
    p[1][1] = (k[1][0] * p_k[0][1]) + ((1 - k[1][1]) * p_k[1][1]);

    return x[0];
	}
};

#endif
