#ifndef __UTOOLS_ANY_TYPE_STORE_H__
#define __UTOOLS_ANY_TYPE_STORE_H__

#include <unordered_map>
#include <memory>
#include <string>

namespace utools::mem
{
    /// @brief 键入标记结构，以便在编译时将类型与唯一ID相关联
    template <typename T>
    struct TypeTag
    {
        static const void *id()
        {
            static const char tag;
            return &tag;
        }
    };

    class AnyMap
    {
    public:
        /// @brief 类型擦除基类
        struct BaseStorage
        {
            virtual ~BaseStorage() = default;
            virtual const void *type() const = 0;
        };

        /// @brief 用于存储特定类型的派生类模板
        template <typename T>
        struct Storage : BaseStorage
        {
            Storage(const T &value) : value(value) {}
            const void *type() const override { return TypeTag<T>::id(); }
            T value;
        };

        /// @brief 键管理类
        class Key
        {
        public:
            template <typename T>
            Key(const T &key) : keyData(std::make_shared<KeyData<T>>(key)), type(TypeTag<T>::id()) {}

            bool operator==(const Key &other) const
            {
                return type == other.type && keyData->equals(*other.keyData);
            }

            struct KeyDataBase
            {
                virtual ~KeyDataBase() = default;
                virtual bool equals(const KeyDataBase &other) const = 0;
                virtual std::size_t hash() const = 0;
            };

            template <typename T>
            struct KeyData : KeyDataBase
            {
                KeyData(const T &key) : key(key) {}

                bool equals(const KeyDataBase &other) const override
                {
                    const auto &otherKey = static_cast<const KeyData &>(other);
                    return key == otherKey.key;
                }

                std::size_t hash() const override
                {
                    return std::hash<T>()(key);
                }

                T key;
            };

            std::size_t hash() const
            {
                return keyData->hash();
            }

        private:
            std::shared_ptr<KeyDataBase> keyData;
            const void *type;
        };

        struct KeyHash
        {
            std::size_t operator()(const Key &k) const
            {
                return k.hash();
            }
        };

        /// @brief 设置一个指定的值
        /// @tparam T 值类型
        /// @tparam K 键类型
        /// @param key 键
        template <typename T, typename K>
        AnyMap &set(const K &key, const T &value)
        {
            data[Key(key)] = std::make_shared<Storage<T>>(value);
            return *this;
        }

        /// @brief 获取一个指定的值
        /// @tparam T 值类型
        /// @tparam K 键类型
        /// @param key 健值
        /// @return 指定key对应的值，如果没有找到会报错
        template <typename T, typename K>
        T get(const K &key) const
        {
            auto it = data.find(Key(key));
            if (it == data.end())
            {
                // throw std::runtime_error("Key not found");
                return T();
            }
            std::shared_ptr<BaseStorage> basePtr = it->second;
            if (basePtr->type() != TypeTag<T>::id())
            {
                // throw std::runtime_error("Type mismatch for key");
                return T();
            }
            return std::static_pointer_cast<Storage<T>>(basePtr)->value;
        }

        /// @brief 获取指定key对应的值，如果key不存在，则返回默认值
        /// @tparam T 值类型
        /// @tparam K 键类型
        /// @param key 键
        /// @param defaule_val 默认值
        /// @return 指定key对应的值，如果key不存在，则返回默认值
        template <typename T, typename K>
        T get(const K &key, const T &defaule_val) const
        {
            auto it = data.find(Key(key));
            if (it == data.end())
            {
                return defaule_val;
            }
            std::shared_ptr<BaseStorage> basePtr = it->second;
            if (basePtr->type() != TypeTag<T>::id())
            {
                return defaule_val;
            }
            return std::static_pointer_cast<Storage<T>>(basePtr)->value;
        }

        /// @brief 判断是否存在某个key
        /// @tparam K
        /// @param key key值
        /// @return true / false
        template <typename K>
        bool has_key(const K &key) const
        {
            return data.find(Key(key)) != data.end();
        }

        /// @brief 按键删除值
        /// @tparam K 键类型
        /// @param key 键
        /// @return void
        template <typename K>
        AnyMap &remove(const K &key)
        {
            auto it = data.find(Key(key));
            if (it != data.end())
            {
                data.erase(it);
            }
            return *this;
        }

        /// @brief 通过key查找对应的值
        /// @tparam T 值类型
        /// @tparam K 键类型
        /// @param key 键
        /// @return 指定key对应的值，如果没有找到会报错
        template <typename T, typename K>
        T &operator[](const K &key)
        {
            return get<T, K>(key);
        }

    private:
        std::unordered_map<Key, std::shared_ptr<BaseStorage>, KeyHash> data;
    };
}

#endif // __UTOOLS_ANY_TYPE_STORE_H__