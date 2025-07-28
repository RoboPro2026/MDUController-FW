/*
 * id_map_control.hpp
 *
 *  Created on: Feb 25, 2024
 *      Author: yaa3k
 */

#ifndef ID_MAP_CONTROL_HPP_
#define ID_MAP_CONTROL_HPP_

#include "Protocol/byte_reader_writer.hpp"
#include <functional>

namespace CommonLib{

	class DataAccessor{
	private:
		std::function<bool(Protocol::ByteReader&)> f_set;
		std::function<bool(Protocol::ByteWriter&)> f_get;
	public:
		DataAccessor(std::function<bool(Protocol::ByteReader&)>&& _write,std::function<bool(Protocol::ByteWriter&)>&&_read):
			f_set(_write),f_get(_read){}

		//byte->ref
		bool set(Protocol::ByteReader& r){ return f_set ? f_set(r) : false; }
		//ref->byte
		bool get(Protocol::ByteWriter& w){ return f_get ? f_get(w) : false; }

		template<class T>
		static DataAccessor generate(T* ref){
			auto readf = [ref](Protocol::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					*ref = val.value();
					return true;
				}else{
					return false;
				}
			};
			auto writef = [ref](Protocol::ByteWriter& w){
				w.write<T>(*ref);
				return true;
			};
			return DataAccessor(readf,writef);
		}

		template<class T>
		static DataAccessor generate(std::function<void(T)> setter,std::function<T(void)> getter){
			auto readf = [setter](Protocol::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					setter(val.value());
					return true;
				}else{
				   return false;
				}
			};
			auto writef = [getter](Protocol::ByteWriter& w){
				w.write(getter());
				return true;
			};
			return DataAccessor(readf,writef);
		}

		template<class T>
		static DataAccessor generate(std::function<void(T)> setter){
			auto readf = [setter](Protocol::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					setter(val.value());
					return true;
				}else{
				   return false;
				}
			};
			return DataAccessor(readf,nullptr);
		}

		template<class T>
		static DataAccessor generate(std::function<T(void)> getter){
			auto writef = [getter](Protocol::ByteWriter& w){
				w.write(getter());
				return true;
			};
			return DataAccessor(nullptr,writef);
		}
	};


	class IDMap{
	public:
		std::unordered_map<size_t, DataAccessor> accessors_map;
		IDMap(std::unordered_map<size_t, DataAccessor>&& _accessors_map):accessors_map(_accessors_map){}

		bool set(int id,Protocol::ByteReader& r){
			auto iter=accessors_map.find(id);
			if (iter!=accessors_map.end()){
				return iter->second.set(r);
			}
			return false;
		}
		bool get(int id,Protocol::ByteWriter& w){
			auto iter=accessors_map.find(id);
			if (iter!=accessors_map.end()){
				return iter->second.get(w);
			}
			return false;
		}
	};

	class IDMapBuilder{
	public:
		std::unordered_map<size_t, DataAccessor> accessors_map;
		IDMapBuilder& add(size_t id,const DataAccessor& c){
			accessors_map.insert(std::pair(id, c)); //どうせ初期化時しか使わないからええやろ，の信仰
			return *this;
		}
		IDMap build(){
		   return IDMap(std::move(accessors_map));
		}
	};


}


#endif /* ID_MAP_CONTROL_HPP_ */
