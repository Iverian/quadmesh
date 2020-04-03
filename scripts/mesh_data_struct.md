# Структура данных сетки

Я думал, на что можно заменить тупорылую структуру сетки, которая есть сейчас

Над сеткой могут выполняться операции (упорядочено по убыванию частоты, примерно):

1. Вставка нового узла
2. Вставка нового элемента (элемент -- набор из 4х узлов)
3. Поиск элементов, в которых находится узел (в сглаживании для отлова случаев, когда узел находится в двух элементах)
4. Замена узла `I` на `J` (по сути выполняется слияние узлов в один и новое значение кладется в ячейку `J`, для склеек)

Сейчас так:

```cpp
using Point = array<double, 3>; // для простоты
using Index = int;

struct Vertex {
    Index index;
    Point value;
    bool is_external;
};

using VtxPtr = list<Vertex>::pointer;
struct Mesh {
    list<Vertex> vertices;
    vector<array<Index, 4>> elements;
    vector<array<Index, 2>> edges;
};
```

Тут можно ускорить (3) добавив `unordered_multimap<Index, Index>` для отображения индекса узла в индекс элемента.

Сейчас операции (1) и (2) выполняются без проблем, можно для ускорения (4) перейти к такой структуре:

```cpp
using Point = array<double, 3>; // для простоты
using Index = int;

static constexpr Index npos(-1);

struct BufItem {
    variant<Vertex, Index> value;
    vector<BufRef> refs;

    operator bool() {
        return holds_alternative<Vertex>(value);
    }
};

using VtxPtr = Vertex*;

struct Mesh {
    using ItemRef = shared_ptr<Index>;
    using Elem = array<BufRef, 4>;
    using Edge = array<BufRef, 2>;

    struct Vertex {
        Point value;
        bool is_external;
        set<BufRef> adjacent;
    };

    vector<BufItem> buf;
    Index last_hole;
    vector<Elem> elems;
    vector<Edge> edges;

    unordered_multimap<Index, Index> elem_reverse_lookup_table;

    vector<Elem> elems_for_vertices(const BufRef& ref)
    {
        if (!ref)
            throw runtime_error("error");
        return elems_for_vertices(*ref);
    }

    vector<Elem> elems_for_vertices(Index index)
    {
        vector<Elem> result;
        result.reserve(4);
        auto range = elem_reverse_lookup_table.equal_range(index);
        for (auto i = range.first; i != range.second; ++i) {
            result.push_back(elems[*i]);
        }
        return result;
    }

    BufRef insert(Vertex obj)
    {
        BufRef result;
        if (last_hole == npos) {
            result = make_shared<Index>(buf.size());
            buf.emplace_back(obj);
        }
        else {
            result = make_shared<Index>(last_hole);
            last_hole = get<Index>(buf[last_hole].value);
            buf[last_hole].value.emplace(obj);
        }
        buf[*result].refs.push_back(result);
        return result;
    }

    BufRef replace(Index from, Index to)
    {
        auto& vf = buf[from];
        auto& vt = buf[to];
        if (!vf.value || !vt.value)
            throw runtime_error("fuck you");

        for (auto& adj: vf.value.adjacent) {
            vt.value.adjacent.insert(adj);
        }
        vf.value.emplace(last_hole);
        last_hole = from;

        for (auto& r: vf.refs) {
            *r = to;
            vt.refs.emplace_back(move(r));
        }
        vf.refs.clear();

        auto range = elem_reverse_lookup_table.equal_range(from);
        for (auto i = range.first; i != range.second; ++i) {
            elem_reverse_lookup_table.emplace(to, i->second);
        }
        elem_reverse_lookup_table.erase(from);

        return vt.refs[0];
    }

    VtxPtr operator[](const BufRef& ref)
    {
        return ref ? &get<Vertex>(buf[*ref].value) : nullptr;
    }

    VtxPtr operator[](const Index& i)
    {
        return &get<Vertex>(buf[i].value);
    }

    const VtxPtr operator[](const BufRef& ref) const
    {
        return ref ? &get<Vertex>(buf[*ref].value) : nullptr;
    }

    const VtxPtr operator[](const Index& i) const
    {
        return &get<Vertex>(buf[i].value);
    }

    BufRef ref(Index i) const noexcept
    {
        return buf[i].refs[0];
    }
};
```
