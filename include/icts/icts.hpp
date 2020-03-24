#ifndef ICTS_HPP
#define ICTS_HPP

#include <vector>

namespace ICT{

	template<typename Mapf>
	class ICTS{
	public:
		ICTS()	{}
		bool search(Mapf mapf, std::vector<std::pair<int, int> > starts, std::pair<int, std::vector<std::pair<int, int> > > *solution){
			
			return true;
		}

	};
}

#endif