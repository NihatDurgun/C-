#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include<Eigen/Eigen>
#include <vector>
using namespace Eigen;
using namespace std;

MatrixXi randomMatrix(int LowValue, int HighValue,int row,int column) {
	double range = HighValue - LowValue;
	MatrixXd m = MatrixXd::Random(row, column);
	m = (m + MatrixXd::Constant(row, column, 1.)) * range / 2.;
	m = (m + MatrixXd::Constant(row, column, LowValue));
	MatrixXi mint = m.cast<int>();
	return mint;
}

VectorXi randomVector(int LowValue, int HighValue, int size) {
	double range = HighValue - LowValue;
	VectorXd m = VectorXd::Random(size);
	m = (m + VectorXd::Constant(size, 1.)) * range / 2.;
	m = (m + VectorXd::Constant(size, LowValue));
	VectorXi mint = m.cast<int>();
	return mint;
}

void sort_vec(const VectorXi& vec, VectorXi& sorted_vec, VectorXi& ind) {
	ind = VectorXi::LinSpaced(vec.size(), 0, vec.size() - 1);
	auto rule = [vec](int i, int j)->bool {
		return vec(i) > vec(j);
	};
	std::sort(ind.data(), ind.data() + ind.size(), rule);
	sorted_vec.resize(vec.size());
	for (int i = 0; i < vec.size(); i++) {
		sorted_vec(i) = vec(ind(i));
	}
}


Eigen::MatrixXi sortMatrix(const Eigen::MatrixXi& original_matrix) {
	Eigen::MatrixXi sorted_matrix(original_matrix.rows(), original_matrix.cols());

	for (int i = 0; i < original_matrix.cols(); i++) {
		VectorXi ind;
		VectorXi sorted_vec;
		sort_vec(original_matrix.col(i), sorted_vec, ind);
		sorted_matrix.col(i) = sorted_vec;
	}

	return sorted_matrix;
}

int main()
{


	//1-)1 ile 100 arasında rastgele tam sayılardan oluşan 5x8 matris tanımlayınız.
	MatrixXi s1 = randomMatrix(1, 100,5,8);
	cout << "Soru 1-)" << endl;
	cout << s1 << endl;
	cout << "\n" << endl;

	//2-)Rastgele değerlerle bir 5x4x3 dizi oluşturun .
	vector<MatrixXi> s2;
	s2.push_back(randomMatrix(1, 100, 5, 4));
	s2.push_back(randomMatrix(1, 100, 5, 4));
	s2.push_back(randomMatrix(1, 100, 5, 4));

	cout << "Soru 2-)" << endl;
	for (auto i = s2.begin(); i != s2.end(); ++i) {
		cout << *i << endl;
	}
	cout << "\n" << endl;

	//3-)Rastgele değerlerle 10x10’luk bir dizi oluşturun ve minimum-maksimum değerleri bulun.
	MatrixXi s3 = randomMatrix(1, 100, 10, 10);
	cout << "Soru 3-)" << endl;
	cout << "Max value:  " << s3.minCoeff() << endl;
	cout << "Min value:  " << s3.maxCoeff() << endl;
	cout << "\n" << endl;

	//4-)5x3 matrisi 3x2 matrisle çarpın. Bu matrislerin elemanlarını karşılıklı olarak çarpın.
	MatrixXi s4_1 = randomMatrix(1, 100, 5, 3);
	MatrixXi s4_2 = randomMatrix(1, 100, 3, 2);
	MatrixXi s4_3 = s4_1 * s4_2;
	cout << "Soru 4-)" << endl;
	cout << s4_3 << endl;
	cout << "\n" << endl;

	//5-)10 boyutunda rastgele vektör oluşturun ve maksimum değeri 0 ile değiştirin
	VectorXi s5 = randomVector(1, 100, 10);
	//cout << s5 << endl;
	int Max = -9999;
	int	IM = -1;
	for (int i = 0; i < 10; i++) {
		if (Max < s5[i]) {
			Max = s5[i];
			IM = i;
		}
	}
	s5[IM] = 0;
	cout << "Soru 5-)" << endl;
	cout << s5 << endl;
	cout << "\n" << endl;

	//6-)100x150 boyutlu bir matrisinin her bir satırındaki değerleri satır numarası olacak şekilde MM değişkeninde tanımlayınız.
	MatrixXi MM = randomMatrix(0, 0, 100, 150);
	for(int x = 0; x < 100;x++){
		MM.row(x) = (x+1) * MatrixXi::Constant(1, 150, 1);
	}
	cout << "Soru 6-)" << endl;
	cout << MM << endl;
	cout << "\n" << endl;
	/*7-)MM matrisinin dört parçaya bölündüğünü düşünün. Sağ alt köşe (25x25) ile sol üst köşeyi(25x25), sağ üst köşe(25x25) ile 
	sol alt köşedeki(25x25) değerlerin yerini değiştirin.*/
	MatrixXi temp = MM.block<25, 25>(75, 1);
	MM.block<25, 25>(75, 1) = MM.block<25, 25>(1, 125);
	MM.block<25, 25>(1, 125) = temp;

	temp = MM.block<25, 25>(1, 1);
	MM.block<25, 25>(1, 1) = MM.block<25, 25>(75, 125);
	MM.block<25, 25>(75, 125) = temp;

	cout << "Soru 7-)" << endl;
	cout << MM << endl;
	cout << "\n" << endl;
	/*8-)10x10 bir matris içinde 1'den 100'e kadar rakamları saklayın. Bu matrisinin transpozunu yine aynı değişken içerisinde saklayın.*/
	MatrixXi s8 = randomMatrix(1, 100, 10, 10);
	//cout << s8 << endl;
	MatrixXi s8_temp = s8.transpose();
	s8 = s8_temp;
	cout << "Soru 8-)" << endl;
	cout << s8 << endl;
	cout << "\n" << endl;

	//9-)10x10 rastgele tam sayılardan oluşan bir matrisin çift sayılı satırları ile çift sayılı sütun elemanlarını yer değiştirin.
	MatrixXi s9 = randomMatrix(1, 100, 10, 10);
	//cout << s9 << endl;
	for (int x = 0; x < 10; x++) {
		if (x % 2 == 0) {
			MatrixXi s9_temp = s9.row(x).transpose();
			MatrixXi s9_temp2 = s9.col(x).transpose();
			s9.row(x) = s9_temp2;
			s9.col(x) = s9_temp;
		}
	}
	cout << "Soru 9-)" << endl;
	cout << s9 << endl;
	cout << "\n" << endl;

	//10-)100x100 rastgele sayılardan oluşan bir matris içerisinde sıfırdan küçük değerleri pozitif değerleri ile değiştirin.
	MatrixXi s10 = randomMatrix(-100, 100, 10, 10);
	//cout << s10 << endl;
	s10 = s10.cwiseAbs();
	
	cout << "Soru 10-)" << endl;
	cout << s10 << endl;
	cout << "\n" << endl;

	//11-)10x10 rastgele tam sayılardan oluşan bir matrisin satırlarındaki değerleri sıralayarak aynı matriste saklayınız.
	MatrixXi s11 = randomMatrix(-100, 100, 10, 10);
	s11 = sortMatrix(s11);

	cout << "Soru 11-)" << endl;
	cout << s11 << endl;
	cout << "\n" << endl;
}
