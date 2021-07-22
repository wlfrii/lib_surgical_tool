/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

#ifndef H3E0DA004_2F41_4495_BDD5_880697092293
#define H3E0DA004_2F41_4495_BDD5_880697092293

template <typename PTR, size_t N>
class ArrayRepo
{
public:
    ArrayRepo()
        : m_count(0)
    {}

    int add(const PTR& item)
    {
        if (m_count >= N)
        {
            return -1;
        }
        m_elements[m_count] = item;
        m_count++;
        return 0;
    }

    const PTR& operator[](size_t index) const
    {
        return m_elements[index];
    }

    PTR& operator[](size_t index)
    {
        return m_elements[index];
    }

    int count() const
    {
        return m_count;
    }

    template<typename KEY>
	PTR getItemBy(KEY key) const
    {
        for(int i = 0; i < m_count; ++i)
        {
            if (m_elements[i]->isMatch(key))
            {
                return m_elements[i];
            }
        }
        return NULL;
    }
private:
    size_t m_count;
    PTR m_elements[N];
};

#define ARRAY_REPO_FOREACH(repo, index) \
   for(index = 0; index < repo.count(); ++index)

#endif /* H3E0DA004_2F41_4495_BDD5_880697092293 */
