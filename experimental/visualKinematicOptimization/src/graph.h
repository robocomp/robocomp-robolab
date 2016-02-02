#include <QStringList>
#include <QTextStream>


class ConnectivityGraph
{
public:
	struct VertexData
	{
		bool valid;
		float pose[3];
		float poseElbow[3];
		std::vector < MotorGoalPositionList > configurations;
		std::size_t id;
		VertexData()
		{
			id = -1;
			valid = false;
			pose[0] = pose[1] = pose[2] = 0;
			poseElbow[0] = poseElbow[1] = poseElbow[2] = 0;
		}
		VertexData(std::size_t i, const float *p)
		{
			id = i;
			for (int j=0; j<3; j++)
				pose[j] = p[0];
		}
		void setPose(const float *p)
		{
			setPose(p[0], p[1], p[2]);
		}
		void setElbowPose(const float *p)
		{
			setElbowPose(p[0], p[1], p[2]);
		}
		void setPose(const float x, const float y, const float z)
		{
			pose[0] = x;
			pose[1] = y;
			pose[2] = z;
			valid = true;
		}
		void setElbowPose(const float x, const float y, const float z)
		{
			poseElbow[0] = x;
			poseElbow[1] = y;
			poseElbow[2] = z;
		}
		float distTo(const float *p)
		{
			if (valid)
				return sqrt( (p[0]-pose[0])*(p[0]-pose[0]) + (p[1]-pose[1])*(p[1]-pose[1]) + (p[2]-pose[2])*(p[2]-pose[2]) );
			return 1000000000000;
		}
	};
	ConnectivityGraph(int32_t size)
	{
		for (int32_t i=0;i<size; i++)
		{
			vertices.push_back(VertexData());
			std::vector<float> eds;
			for (int32_t j=0;j<size; j++)
			{
				eds.push_back(DJ_INFINITY);
			}
			edges.push_back(eds);
		}
	}

	ConnectivityGraph(QString path)
	{
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			printf("Can't open file\n");
			throw 1;
		}

		QTextStream in(&file);
		int lineN = 0;
		while (!in.atEnd())
		{
			QString line = in.readLine();
			if (line[0] == 'V')
			{
				addVertex(ConnectivityGraph::VertexData());
				QStringList parts = line.split("_");
				vertices[size()-1].id = parts[1].toInt();
				vertices[size()-1].setPose(     parts[2].toFloat(), parts[3].toFloat(), parts[4].toFloat());
				vertices[size()-1].setElbowPose(parts[5].toFloat(), parts[6].toFloat(), parts[7].toFloat());

// 				printf("parts %d\n", parts.size());
				if (parts.size()>8)
				{
					vertices[size()-1].configurations.resize(1);
					for (int m=8; m<parts.size(); m++)
					{
						QStringList goal = parts[m].split(":");
						MotorGoalPosition mgp;
						mgp.position = goal[1].toFloat();
						mgp.maxSpeed = 2.;
						mgp.name = goal[0].toStdString();
						vertices[size()-1].configurations[0].push_back(mgp);
					}
				}
			}
			else
			{
				QStringList parts = line.split("_");
				int a = parts[0].toInt();
				int b = parts[1].toInt();
				float peso = parts[2].toFloat();
				edges[a][b] = peso;
			}
			lineN++;
		}
	}

	bool save(QString path)
	{
		QFile file(path);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
			return false;

		QTextStream out(&file);
		for (uint vid=0; vid<vertices.size(); vid++)
		{
			out << "V_";
			out << vertices[vid].id << "_";
			out << vertices[vid].pose[0]      << "_" << vertices[vid].pose[1]      << "_" << vertices[vid].pose[2] << "_";
			out << vertices[vid].poseElbow[0] << "_" << vertices[vid].poseElbow[1] << "_" << vertices[vid].poseElbow[2];
			if (vertices[vid].configurations.size()>0)
			{
				for (auto motor : vertices[vid].configurations[0])
				{
					out << "_" << QString::fromStdString(motor.name) << ":" << motor.position;
				}
			}
			out << "\n";
		}

		for (uint v1=0; v1<edges.size(); v1++)
		{
			for (uint v2=0; v2<edges[v1].size(); v2++)
			{
				out << v1 << "_" << v2 << "_" << edges[v1][v2] << "\n";
			}
		}
		return true;
	}

	void addVertex(const VertexData &v)
	{
		// Add vertex
		vertices.push_back(v);
		// Add edges for existing nodes
		for (uint32_t j=0;j<vertices.size()-1; j++)
		{
			edges[j].push_back(DJ_INFINITY);
		}
		// Add edges for the new node
		std::vector<float> eds;
		for (uint32_t j=0;j<vertices.size(); j++)
		{
			eds.push_back(DJ_INFINITY);
		}
		edges.push_back(eds);
	}

	int size()
	{
		return vertices.size();
	}



	std::vector<VertexData> vertices;
	std::vector< std::vector< float > > edges;

	void add_edge(int a, int b, float dist)
	{
		edges[a][b] = dist;
		edges[b][a] = dist;
	}

	void add_configurationToNode(int node, MotorGoalPositionList gpl)
	{
		vertices[node].configurations.push_back(gpl);
	}

	int path(int source, int dest, std::vector<int> &path)
	{
		Dijkstra d = Dijkstra(&edges);
		d.calculateDistance(source);
		return d.go(dest, path);
	}


	int getCloserTo(float x, float y, float z)
	{
		float p[3];
		p[0] = x;
		p[1] = y;
		p[2] = z;
		return getCloserTo(p);
	}

	int getCloserTo(float *p)
	{
		float minDist = -1;
		int32_t minIndex = -1;

		for (uint i=0; i<vertices.size(); i++)
		{
			if (vertices[i].valid and vertices[i].configurations.size() > 0)
			{
				const float d = vertices[i].distTo(p);
				if (minDist<0 or minDist>d)
				{
					minDist = d;
					minIndex = i;
				}
			}
		}
		return minIndex;
	}
};

